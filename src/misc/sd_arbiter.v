//
// sd_arbiter.v - share the sd_card core port between several requesters
//
// sd_card.v serves one core request at a time, but its busy, done and
// byte strobe outputs don't say which requester they belong to. This
// module grants the card to one requester at a time (lowest index first),
// presents that requester's sector and write data, and routes busy, done
// and the byte strobe only to that requester.
//
// Requesters keep their rd/wr request high until they see busy, as
// before. A requester that drops its request before busy (e.g. a floppy
// reset by a disk change) is treated as cancelled: the transfer may
// already be on its way, so it runs to completion, but the requester
// sees none of it.
//
// Requester index = sd_card image index:
//   0: floppy 1, 1: floppy 2, 2: hdd, 3: rom loader, 4: palette loader
//

module sd_arbiter (
    input             clk,
    input             rstn,

    // requesters
    input      [4:0]  req_rd,
    input      [4:0]  req_wr,
    input      [31:0] req_lba0,
    input      [31:0] req_lba1,
    input      [31:0] req_lba2,
    input      [31:0] req_lba3,
    input      [31:0] req_lba4,
    input      [7:0]  req_data0,  // write data, valid while granted
    input      [7:0]  req_data1,
    input      [7:0]  req_data2,
    output     [4:0]  req_busy,
    output     [4:0]  req_done,
    output     [4:0]  req_strobe,

    // sd_card core port
    output     [7:0]  sd_rstart,
    output     [7:0]  sd_wstart,
    output reg [31:0] sd_sector,
    output reg [7:0]  sd_inbyte,
    input             sd_busy,
    input             sd_done,
    input             sd_strobe
);

localparam [1:0] IDLE = 2'd0,  // waiting for a request
                 REQ  = 2'd1,  // request presented, waiting for busy
                 XFER = 2'd2;  // transfer running, waiting for busy to drop

reg  [1:0] state;
reg  [2:0] grant;
reg        g_rd, g_wr;
reg        cancelled;

wire [4:0] req_any = req_rd | req_wr;

// lowest pending index wins
wire [2:0] pick = req_any[0] ? 3'd0 :
                  req_any[1] ? 3'd1 :
                  req_any[2] ? 3'd2 :
                  req_any[3] ? 3'd3 : 3'd4;

always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        state     <= IDLE;
        grant     <= 3'd0;
        g_rd      <= 1'b0;
        g_wr      <= 1'b0;
        cancelled <= 1'b0;
        sd_sector <= 32'd0;
    end else begin
        case(state)
            IDLE:
                if(|req_any) begin
                    grant     <= pick;
                    g_rd      <= req_rd[pick];
                    g_wr      <= req_wr[pick];
                    cancelled <= 1'b0;
                    case(pick)
                        3'd0:    sd_sector <= req_lba0;
                        3'd1:    sd_sector <= req_lba1;
                        3'd2:    sd_sector <= req_lba2;
                        3'd3:    sd_sector <= req_lba3;
                        default: sd_sector <= req_lba4;
                    endcase
                    state <= REQ;
                end

            REQ:
                if(sd_busy)
                    state <= XFER;
                else if(!req_any[grant])
                    cancelled <= 1'b1;

            XFER:
                if(!sd_busy)
                    state <= IDLE;

            default:
                state <= IDLE;
        endcase
    end
end

// hold the latched request until the transfer starts, even if the
// requester cancels, so the MCU path still sees a consistent request
wire       presenting = (state == REQ);
wire [7:0] grant_bit  = 8'd1 << grant;

assign sd_rstart = (presenting && g_rd) ? grant_bit : 8'd0;
assign sd_wstart = (presenting && g_wr) ? grant_bit : 8'd0;

// sd_busy, sd_done and sd_strobe are only active during core transfers,
// and only the granted one can be in flight
wire       owned = (state != IDLE) && !cancelled;
wire [4:0] mask  = owned ? grant_bit[4:0] : 5'd0;

assign req_busy   = sd_busy   ? mask : 5'd0;
assign req_done   = sd_done   ? mask : 5'd0;
assign req_strobe = sd_strobe ? mask : 5'd0;

always @(*) begin
    case(grant)
        3'd0:    sd_inbyte = req_data0;
        3'd1:    sd_inbyte = req_data1;
        3'd2:    sd_inbyte = req_data2;
        default: sd_inbyte = 8'd0;
    endcase
end

endmodule
