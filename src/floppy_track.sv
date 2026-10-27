//
// Apple ][ track read/write interface to MiST
//
// Based on the work of
// Copyright (c) 2016 Sorgelig
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the Lesser GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// .dsk/.do/.po support: a track is 8 SD sectors of sector data. Each one
// is read into the staging area of the track buffer and encoded into disk
// bytes by nib_codec. Saving scans the track the Apple has written,
// decodes the sectors back and writes them. A sector that doesn't decode
// keeps its old contents on the SD card.
//
/////////////////////////////////////////////////////////////////////////

module floppy_track
(
	input         clk,  // apple IIe domain
	input         clk2, // system controller domain
	input         reset,

	output [31:0] sd_lba,
	output reg    sd_rd,
	output reg    sd_wr,
	input         sd_ack,

	input   [8:0] sd_buff_addr,
	input   [7:0] sd_buff_dout,
	output  [7:0] sd_buff_din,
	input         sd_buff_wr,

	input         change,
	input         mount,
	input   [1:0] format,   // with mount: 0 .nib, 1 .dsk/.do (DOS order), 2 .po (ProDOS order)
	input   [5:0] track,    // Apple
	output        ready,    // Apple
	input         active,   // Apple

	input  [12:0] ram_addr, // Apple DPRAM
	output  [7:0] ram_do,   // Apple DPRAM
	input   [7:0] ram_di,   // Apple DPRAM
	input         ram_we,   // Apple logic + DPRAM
	output        busy      // Apple
);

reg  [31:0] lba;
reg   [3:0] rel_lba;
reg ram_weD,ram_weD2;
reg [5:0] trackD, trackD2;
reg activeD, activeD2;
reg readyi, readyD, readyD2;
reg busyi, busyD, busyD2;

assign ready = readyD2;
assign busy = busyD2;
assign sd_lba = lba;
reg   [1:0] fmt;

// physical sector holding 256 byte block c (0-15) of an 8 SD sector track
function [3:0] phys(input [1:0] f, input [3:0] c);
	reg [3:0] p;
	begin
		if(f == 2'd2)
			// ProDOS order: physical 0,2,4,..14 hold blocks 0-7, 1,3,..15 hold 8-15
			p = { c[2:0], c[3] };
		else case(c)
			// DOS 3.3 order
			4'd0:  p = 4'd0;   4'd1:  p = 4'd13;  4'd2:  p = 4'd11;  4'd3:  p = 4'd9;
			4'd4:  p = 4'd7;   4'd5:  p = 4'd5;   4'd6:  p = 4'd3;   4'd7:  p = 4'd1;
			4'd8:  p = 4'd14;  4'd9:  p = 4'd12;  4'd10: p = 4'd10;  4'd11: p = 4'd8;
			4'd12: p = 4'd6;   4'd13: p = 4'd4;   4'd14: p = 4'd2;   default: p = 4'd15;
		endcase
		phys = p;
	end
endfunction

// .dsk/.po sequencer
localparam [3:0]
	D_IDLE = 4'd0,
	D_LRD  = 4'd1,   // load: SD sector read
	D_LENC0= 4'd2,   // load: encode first half
	D_LENC1= 4'd3,   // load: encode second half
	D_PAD  = 4'd4,   // load: fill the gap after the last sector
	D_SCAN = 4'd5,   // save: find the sectors on the track
	D_DEC0 = 4'd6,   // save: decode first half
	D_DEC1 = 4'd7,   // save: decode second half
	D_RMW  = 4'd8,   // save: one half bad, read the old SD sector
	D_RE   = 4'd10,  // save: decode the good half again over the old data
	D_WR   = 4'd11,  // save: SD sector write
	D_NEXT = 4'd12;  // save: next SD sector, or done

localparam [1:0] C_ENCODE = 2'd0, C_PAD = 2'd1, C_SCAN = 2'd2, C_DECODE = 2'd3;

reg  [3:0] dst;
reg  [2:0] dsec;      // SD sector within the track
reg        ok0;
reg        redo;      // half to decode again after a read-modify-write
reg        c_start, c_half, c_reset;
reg  [1:0] c_cmd;
reg  [3:0] c_psec;
wire       c_busy, c_done, c_ok;
wire [12:0] c_addr;
wire  [7:0] c_wdata;
wire        c_we;

reg  [5:0] cur_track;

always @(posedge clk2) begin
	reg old_ack;
	reg old_change;
	reg saving;
	reg dirty;
	reg old_reset;
	reg reload;

	old_change <= change;
	old_ack <= sd_ack;
	old_reset <= reset;
	c_start <= 0;
	c_reset <= 0;

	if(sd_ack) {sd_rd,sd_wr} <= 0;

	if(readyi && ram_weD2) dirty <= 1;

	if(~old_change & change) begin
		readyi <= mount;
		fmt <= format;
		cur_track <= 'b111111;
		busyi  <= 0;
		sd_rd <= 0;
		sd_wr <= 0;
		saving<= 0;
		dirty <= 0;
		reload <= 0;
		dst <= D_IDLE;
		c_reset <= 1;
	end
	else begin
	// reload the track once reset is released; a dirty track is
	// saved first instead of being discarded
	if(old_reset & ~reset) reload <= 1;

	if(busyi && fmt == 2'd0) begin
		if(old_ack && ~sd_ack) begin
			if(rel_lba != 4'd12) begin
				lba <= lba + 1'd1;
				rel_lba <= rel_lba + 1'd1;
				if(saving) sd_wr <= 1;
					else sd_rd <= 1;
			end
			else
			if(saving && (cur_track != trackD2)) begin
				saving <= 0;
				cur_track <= trackD2;
				rel_lba <= 0;
                lba <= trackD2 * 8'd13; //track size = 1a00h = 13*512
				sd_rd <= 1;
				reload <= 0;
			end
			else
			begin
				busyi <= 0;
				dirty <= 0;
			end
		end
	end
	else
	if(busyi) begin
		// .dsk/.po: every SD transfer goes through the staging area
		rel_lba <= 4'd13;
		case(dst)
		D_LRD:
			if(old_ack && ~sd_ack) begin
				c_cmd <= C_ENCODE; c_half <= 0; c_psec <= phys(fmt, { dsec, 1'b0 }); c_start <= 1;
				dst <= D_LENC0;
			end
		D_LENC0:
			if(c_done) begin
				c_cmd <= C_ENCODE; c_half <= 1; c_psec <= phys(fmt, { dsec, 1'b1 }); c_start <= 1;
				dst <= D_LENC1;
			end
		D_LENC1:
			if(c_done) begin
				if(dsec == 3'd7) begin
					c_cmd <= C_PAD; c_start <= 1;
					dst <= D_PAD;
				end else begin
					dsec <= dsec + 1'd1;
					lba <= { cur_track, dsec + 3'd1 };
					sd_rd <= 1;
					dst <= D_LRD;
				end
			end
		D_PAD:
			if(c_done) begin
				busyi <= 0;
				dirty <= 0;
				dst <= D_IDLE;
			end
		D_SCAN:
			if(c_done) begin
				dsec <= 3'd0;
				c_cmd <= C_DECODE; c_half <= 0; c_psec <= phys(fmt, 4'd0); c_start <= 1;
				dst <= D_DEC0;
			end
		D_DEC0:
			if(c_done) begin
				ok0 <= c_ok;
				c_cmd <= C_DECODE; c_half <= 1; c_psec <= phys(fmt, { dsec, 1'b1 }); c_start <= 1;
				dst <= D_DEC1;
			end
		D_DEC1:
			if(c_done) begin
				lba <= { cur_track, dsec };
				if(ok0 && c_ok) begin
					sd_wr <= 1;
					dst <= D_WR;
				end else if(ok0 || c_ok) begin
					// keep the old data of the bad half: read, decode the good half over it
					redo <= c_ok;
					sd_rd <= 1;
					dst <= D_RMW;
				end else
					dst <= D_NEXT;              // nothing decoded: leave the SD sector alone
			end
		D_RMW:
			if(old_ack && ~sd_ack) begin
				c_cmd <= C_DECODE; c_half <= redo; c_psec <= phys(fmt, { dsec, redo }); c_start <= 1;
				dst <= D_RE;
			end
		D_RE:
			if(c_done) begin
				sd_wr <= 1;
				dst <= D_WR;
			end
		D_WR:
			if(old_ack && ~sd_ack)
				dst <= D_NEXT;
		D_NEXT:
			begin
				if(dsec != 3'd7) begin
					dsec <= dsec + 1'd1;
					c_cmd <= C_DECODE; c_half <= 0; c_psec <= phys(fmt, { dsec + 3'd1, 1'b0 }); c_start <= 1;
					dst <= D_DEC0;
				end else if(cur_track != trackD2) begin
					saving <= 0;
					cur_track <= trackD2;
					dsec <= 3'd0;
					lba <= { trackD2, 3'd0 };
					sd_rd <= 1;
					reload <= 0;
					dst <= D_LRD;
				end else begin
					busyi <= 0;
					dirty <= 0;
					dst <= D_IDLE;
				end
			end
		default: ;
		endcase
	end
	else
	if(readyi && ((cur_track != trackD2) || (old_change && ~change) || (dirty && ~activeD2) || reload))
		if (dirty && cur_track != 'b111111) begin
			saving <= 1;
			busyi <= 1;
			if(fmt == 2'd0) begin
				lba <= cur_track * 8'd13;
				rel_lba <= 0;
				sd_wr <= 1;
			end else begin
				c_cmd <= C_SCAN; c_start <= 1;
				dst <= D_SCAN;
			end
		end
		else
		begin
			saving <= 0;
			cur_track <= trackD2;
			busyi <= 1;
			dirty <= 0;
			reload <= 0;
			sd_rd <= 1;
			if(fmt == 2'd0) begin
				rel_lba <= 0;
				lba <= trackD2 * 8'd13; //track size = 1a00h
			end else begin
				rel_lba <= 4'd13;
				dsec <= 3'd0;
				lba <= { trackD2, 3'd0 };
				dst <= D_LRD;
			end
		end
	end
end

nib_codec codec(
	.clk(clk2),
	.reset(c_reset),
	.start(c_start),
	.cmd(c_cmd),
	.half(c_half),
	.psec(c_psec),
	.track(cur_track),
	.busy(c_busy),
	.done(c_done),
	.ok(c_ok),
	.addr(c_addr),
	.wdata(c_wdata),
	.we(c_we),
	.rdata(sd_buff_din)
);

// port A: SD transfers, or the codec while it runs
Gowin_DPB_trkbuf trkbuf_inst(
	.clka(clk2), // system domain
	.dina(c_busy ? c_wdata : sd_buff_dout),
	.douta(sd_buff_din),
	.ada(c_busy ? c_addr : {rel_lba, sd_buff_addr}),
 	.wrea(c_busy ? c_we : (sd_buff_wr & sd_ack)),
	.ocea(1'b1),
	.cea(1'b1),
	.reseta(1'b0),
	.clkb(clk), 
	.dinb(ram_di),
	.doutb(ram_do),
	.adb(ram_addr),
	.wreb(ram_we),
	.oceb(1'b1),
	.ceb(1'b1),
	.resetb(1'b0)
);


always@(posedge clk2) begin : FDD_IN
// input synchronisers
// bring flags from core clock domain into system clock domain
 { ram_weD ,ram_weD2 } <= { ram_we, ram_weD };
 { trackD ,trackD2 } <= { track, trackD };
 { activeD ,activeD2 } <= { active, activeD };
end


always@(posedge clk) begin : FDD_OUT
// output synchronisers
// bring flags from system clock domain into core clock domain
 { readyD ,readyD2 } <= { readyi, readyD };
 { busyD ,busyD2 } <= { busyi, busyD };
end


endmodule
