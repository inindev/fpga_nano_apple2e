//
// nib_codec.sv - 16-sector Apple II track encoder/decoder for .dsk/.po images
//
// Works on the floppy track buffer (8K, port A), which holds the track as
// disk bytes at 0000-19FF, the way the Disk II reads and writes it. The
// spare space above is used for sector data:
//   1A00-1BFF  staging: one 512 byte SD sector = two 256 byte disk sectors
//   1C00-1C55  scratch for the 86 low-bit values while decoding
//
// Commands (one at a time, start -> done):
//   ENCODE  staging half -> physical sector slot psec*409, laid out as
//           38 sync, address field, 8 sync, data field (same as dsk2nib)
//   PAD     fill 1990-19FF (after the 16 slots) with sync bytes
//   SCAN    find every sector's address field and the data field after
//           it, wherever the Apple has written them; must run before DECODE
//   DECODE  data field of psec -> staging half; ok = checksum good
//
// The buffer port has one clock of read latency: data for an address
// set on one clock is valid two clocks later.
//

module nib_codec (
    input             clk,
    input             reset,    // abort any command
    input             start,
    input       [1:0] cmd,
    input             half,     // staging half
    input       [3:0] psec,     // physical sector
    input       [5:0] track,
    output reg        busy,
    output reg        done,     // one clock pulse
    output reg        ok,       // DECODE result

    output reg [12:0] addr,
    output reg  [7:0] wdata,
    output reg        we,
    input       [7:0] rdata
);

localparam [1:0] CMD_ENCODE = 2'd0, CMD_PAD = 2'd1, CMD_SCAN = 2'd2, CMD_DECODE = 2'd3;

localparam [12:0] TRACK_LEN = 13'd6656;  // disk bytes per track
localparam [12:0] SLOT_LEN  = 13'd409;   // disk bytes per sector slot
localparam [12:0] PAD_START = 13'd6544;  // 16 * 409
localparam [12:0] STAGE     = 13'h1A00;
localparam [12:0] SCRATCH   = 13'h1C00;
localparam [7:0]  VOLUME    = 8'd254;
localparam [13:0] SCAN_LEN  = 14'd7168;  // a whole turn plus a sector, for fields across the wrap
localparam [6:0]  DATA_WINDOW = 7'd64;   // max bytes from address field to data prologue

// 6-and-2 write translate table
function [7:0] gcr(input [5:0] v);
    case(v)
        6'h00: gcr = 8'h96; 6'h01: gcr = 8'h97; 6'h02: gcr = 8'h9A; 6'h03: gcr = 8'h9B;
        6'h04: gcr = 8'h9D; 6'h05: gcr = 8'h9E; 6'h06: gcr = 8'h9F; 6'h07: gcr = 8'hA6;
        6'h08: gcr = 8'hA7; 6'h09: gcr = 8'hAB; 6'h0A: gcr = 8'hAC; 6'h0B: gcr = 8'hAD;
        6'h0C: gcr = 8'hAE; 6'h0D: gcr = 8'hAF; 6'h0E: gcr = 8'hB2; 6'h0F: gcr = 8'hB3;
        6'h10: gcr = 8'hB4; 6'h11: gcr = 8'hB5; 6'h12: gcr = 8'hB6; 6'h13: gcr = 8'hB7;
        6'h14: gcr = 8'hB9; 6'h15: gcr = 8'hBA; 6'h16: gcr = 8'hBB; 6'h17: gcr = 8'hBC;
        6'h18: gcr = 8'hBD; 6'h19: gcr = 8'hBE; 6'h1A: gcr = 8'hBF; 6'h1B: gcr = 8'hCB;
        6'h1C: gcr = 8'hCD; 6'h1D: gcr = 8'hCE; 6'h1E: gcr = 8'hCF; 6'h1F: gcr = 8'hD3;
        6'h20: gcr = 8'hD6; 6'h21: gcr = 8'hD7; 6'h22: gcr = 8'hD9; 6'h23: gcr = 8'hDA;
        6'h24: gcr = 8'hDB; 6'h25: gcr = 8'hDC; 6'h26: gcr = 8'hDD; 6'h27: gcr = 8'hDE;
        6'h28: gcr = 8'hDF; 6'h29: gcr = 8'hE5; 6'h2A: gcr = 8'hE6; 6'h2B: gcr = 8'hE7;
        6'h2C: gcr = 8'hE9; 6'h2D: gcr = 8'hEA; 6'h2E: gcr = 8'hEB; 6'h2F: gcr = 8'hEC;
        6'h30: gcr = 8'hED; 6'h31: gcr = 8'hEE; 6'h32: gcr = 8'hEF; 6'h33: gcr = 8'hF2;
        6'h34: gcr = 8'hF3; 6'h35: gcr = 8'hF4; 6'h36: gcr = 8'hF5; 6'h37: gcr = 8'hF6;
        6'h38: gcr = 8'hF7; 6'h39: gcr = 8'hF9; 6'h3A: gcr = 8'hFA; 6'h3B: gcr = 8'hFB;
        6'h3C: gcr = 8'hFC; 6'h3D: gcr = 8'hFD; 6'h3E: gcr = 8'hFE; default: gcr = 8'hFF;
    endcase
endfunction

// read translate table: {valid, value}
function [6:0] ungcr(input [7:0] b);
    case(b)
        8'h96: ungcr = 7'h40; 8'h97: ungcr = 7'h41; 8'h9A: ungcr = 7'h42; 8'h9B: ungcr = 7'h43;
        8'h9D: ungcr = 7'h44; 8'h9E: ungcr = 7'h45; 8'h9F: ungcr = 7'h46; 8'hA6: ungcr = 7'h47;
        8'hA7: ungcr = 7'h48; 8'hAB: ungcr = 7'h49; 8'hAC: ungcr = 7'h4A; 8'hAD: ungcr = 7'h4B;
        8'hAE: ungcr = 7'h4C; 8'hAF: ungcr = 7'h4D; 8'hB2: ungcr = 7'h4E; 8'hB3: ungcr = 7'h4F;
        8'hB4: ungcr = 7'h50; 8'hB5: ungcr = 7'h51; 8'hB6: ungcr = 7'h52; 8'hB7: ungcr = 7'h53;
        8'hB9: ungcr = 7'h54; 8'hBA: ungcr = 7'h55; 8'hBB: ungcr = 7'h56; 8'hBC: ungcr = 7'h57;
        8'hBD: ungcr = 7'h58; 8'hBE: ungcr = 7'h59; 8'hBF: ungcr = 7'h5A; 8'hCB: ungcr = 7'h5B;
        8'hCD: ungcr = 7'h5C; 8'hCE: ungcr = 7'h5D; 8'hCF: ungcr = 7'h5E; 8'hD3: ungcr = 7'h5F;
        8'hD6: ungcr = 7'h60; 8'hD7: ungcr = 7'h61; 8'hD9: ungcr = 7'h62; 8'hDA: ungcr = 7'h63;
        8'hDB: ungcr = 7'h64; 8'hDC: ungcr = 7'h65; 8'hDD: ungcr = 7'h66; 8'hDE: ungcr = 7'h67;
        8'hDF: ungcr = 7'h68; 8'hE5: ungcr = 7'h69; 8'hE6: ungcr = 7'h6A; 8'hE7: ungcr = 7'h6B;
        8'hE9: ungcr = 7'h6C; 8'hEA: ungcr = 7'h6D; 8'hEB: ungcr = 7'h6E; 8'hEC: ungcr = 7'h6F;
        8'hED: ungcr = 7'h70; 8'hEE: ungcr = 7'h71; 8'hEF: ungcr = 7'h72; 8'hF2: ungcr = 7'h73;
        8'hF3: ungcr = 7'h74; 8'hF4: ungcr = 7'h75; 8'hF5: ungcr = 7'h76; 8'hF6: ungcr = 7'h77;
        8'hF7: ungcr = 7'h78; 8'hF9: ungcr = 7'h79; 8'hFA: ungcr = 7'h7A; 8'hFB: ungcr = 7'h7B;
        8'hFC: ungcr = 7'h7C; 8'hFD: ungcr = 7'h7D; 8'hFE: ungcr = 7'h7E; 8'hFF: ungcr = 7'h7F;
        default: ungcr = 7'h00;
    endcase
endfunction

// low two bits of a data byte are stored swapped
function [1:0] sw(input [7:0] b);
    sw = { b[0], b[1] };
endfunction

// address field byte n (0-7): volume, track, sector, checksum, each as
// two "4 and 4" bytes: odd bits, then even bits, both with every other bit set
function [7:0] fm(input [7:0] v, input lo);
    fm = lo ? (v | 8'hAA) : ((v >> 1) | 8'hAA);
endfunction

// fixed part of a slot: bytes 0-62 (up to the data prologue)
function [7:0] slot_hdr(input [5:0] n, input [7:0] trk, input [7:0] sec);
    reg [7:0] chk;
    begin
        chk = VOLUME ^ trk ^ sec;
        case(n)
            6'd38: slot_hdr = 8'hD5;
            6'd39: slot_hdr = 8'hAA;
            6'd40: slot_hdr = 8'h96;
            6'd41: slot_hdr = fm(VOLUME, 1'b0);
            6'd42: slot_hdr = fm(VOLUME, 1'b1);
            6'd43: slot_hdr = fm(trk, 1'b0);
            6'd44: slot_hdr = fm(trk, 1'b1);
            6'd45: slot_hdr = fm(sec, 1'b0);
            6'd46: slot_hdr = fm(sec, 1'b1);
            6'd47: slot_hdr = fm(chk, 1'b0);
            6'd48: slot_hdr = fm(chk, 1'b1);
            6'd49: slot_hdr = 8'hDE;
            6'd50: slot_hdr = 8'hAA;
            6'd51: slot_hdr = 8'hEB;
            6'd60: slot_hdr = 8'hD5;
            6'd61: slot_hdr = 8'hAA;
            6'd62: slot_hdr = 8'hAD;
            default: slot_hdr = 8'hFF;   // 0-37 and 52-59: sync
        endcase
    end
endfunction

function [12:0] wrap_inc(input [12:0] a);
    wrap_inc = (a == TRACK_LEN - 1'd1) ? 13'd0 : a + 1'd1;
endfunction

localparam [4:0]
    IDLE      = 5'd0,
    RD_WAIT   = 5'd1,   // read issued, data valid next clock in "ret"
    E_HDR     = 5'd2,
    E_AUX0    = 5'd3,
    E_AUX1    = 5'd4,
    E_AUX2    = 5'd5,
    E_AUXW    = 5'd6,
    E_MAIN    = 5'd7,
    E_MAINW   = 5'd8,
    E_TAIL    = 5'd9,
    PAD       = 5'd10,
    SCAN      = 5'd11,
    D_AUX     = 5'd12,
    D_AUXW    = 5'd13,
    D_MAIN    = 5'd14,
    D_MAUX    = 5'd15,
    D_MAINW   = 5'd16,
    D_CK      = 5'd17,
    FINISH    = 5'd18;

reg  [4:0] st, ret;
reg [12:0] wptr;        // encode write pointer / decode read pointer
reg  [8:0] n;           // byte counter
reg  [5:0] last;        // running checksum
reg  [7:0] b0, b1;      // encode: low-bit source bytes
reg  [5:0] v6;          // decode: high six bits of the current byte
reg        bad;
reg  [1:0] c_cmd;
reg        c_half;
reg  [3:0] c_sec;
reg  [5:0] c_track;

// scan results
reg [12:0] dofs [0:15]; // offset of the first data byte after D5 AA AD
reg [15:0] found;

// scan pipeline and parser
reg [13:0] scnt;
reg [12:0] rptr, o1, o2;
reg        v1, v2;
reg  [7:0] sh1, sh0;    // previous two bytes
reg  [1:0] pst;         // 0 search address, 1 address bytes, 2 search data
reg  [2:0] acnt;
reg  [7:0] ab[0:6];
reg  [3:0] asec;
reg        aok;
reg  [6:0] win;

wire [12:0] slot = c_sec * SLOT_LEN;
wire [12:0] stg  = STAGE | { 4'd0, c_half, 8'd0 };

// decode: which low-bit value and which bit pair belong to data byte n
wire [8:0] aux_n  = (n < 9'd86) ? n : (n < 9'd172) ? n - 9'd86 : n - 9'd172;
wire [6:0] aux_k  = aux_n[6:0];
wire [1:0] aux_sh = (n < 9'd86) ? 2'd0 : (n < 9'd172) ? 2'd1 : 2'd2;

wire [6:0] ug = ungcr(rdata);

// address field decode from the 8 bytes (last one is rdata)
wire [7:0] f_vol = { ab[0][6:0], 1'b1 } & ab[1];
wire [7:0] f_trk = { ab[2][6:0], 1'b1 } & ab[3];
wire [7:0] f_sec = { ab[4][6:0], 1'b1 } & ab[5];
wire [7:0] f_chk = { ab[6][6:0], 1'b1 } & rdata;

always @(posedge clk) begin
    done <= 1'b0;
    we   <= 1'b0;

    if(reset) begin
        st   <= IDLE;
        busy <= 1'b0;
    end else case(st)

    IDLE:
        if(start) begin
            busy    <= 1'b1;
            c_cmd   <= cmd;
            c_half  <= half;
            c_sec   <= psec;
            c_track <= track;
            n       <= 9'd0;
            last    <= 6'd0;
            bad     <= 1'b0;
            ok      <= 1'b0;
            case(cmd)
                CMD_ENCODE: st <= E_HDR;
                CMD_PAD:  begin wptr <= PAD_START; st <= PAD; end
                CMD_SCAN: begin
                    found <= 16'd0;
                    scnt  <= 14'd0;
                    rptr  <= 13'd0;
                    v1    <= 1'b0;
                    v2    <= 1'b0;
                    pst   <= 2'd0;
                    sh0   <= 8'h00;
                    sh1   <= 8'h00;
                    st    <= SCAN;
                end
                CMD_DECODE:
                    if(found[psec]) begin
                        wptr <= dofs[psec];
                        st   <= D_AUX;
                    end else
                        st <= FINISH;
            endcase
        end

    RD_WAIT:
        st <= ret;

    // ---------------- encode ----------------

    E_HDR: begin
        addr  <= slot + n;
        wdata <= slot_hdr(n[5:0], { 2'd0, c_track }, { 4'd0, c_sec });
        we    <= 1'b1;
        if(n == 9'd62) n <= 9'd0;
        else           n <= n + 1'd1;
        if(n == 9'd62) st <= E_AUX0;
    end

    // low-bit value k from data bytes k, k+86 and k+172 (wrapping to 0, 1)
    E_AUX0: begin addr <= stg | n;                         ret <= E_AUX1; st <= RD_WAIT; end
    E_AUX1: begin b0 <= rdata; addr <= stg | (n + 9'd86);  ret <= E_AUX2; st <= RD_WAIT; end
    E_AUX2: begin b1 <= rdata; addr <= stg | ((n + 9'd172) & 9'hFF); ret <= E_AUXW; st <= RD_WAIT; end
    E_AUXW: begin : aux_w
        reg [5:0] v;
        v = { sw(rdata), sw(b1), sw(b0) };
        addr  <= slot + 13'd63 + n;
        wdata <= gcr(v ^ last);
        we    <= 1'b1;
        last  <= v;
        if(n == 9'd85) begin n <= 9'd0; st <= E_MAIN; end
        else           begin n <= n + 1'd1; st <= E_AUX0; end
    end

    E_MAIN: begin addr <= stg | n; ret <= E_MAINW; st <= RD_WAIT; end
    E_MAINW: begin
        addr  <= slot + 13'd149 + n;
        wdata <= gcr(rdata[7:2] ^ last);
        we    <= 1'b1;
        last  <= rdata[7:2];
        if(n == 9'd255) begin n <= 9'd0; st <= E_TAIL; end
        else            begin n <= n + 1'd1; st <= E_MAIN; end
    end

    // checksum and data epilogue: slot bytes 405-408
    E_TAIL: begin
        addr <= slot + 13'd405 + n;
        case(n[1:0])
            2'd0: wdata <= gcr(last);
            2'd1: wdata <= 8'hDE;
            2'd2: wdata <= 8'hAA;
            2'd3: wdata <= 8'hEB;
        endcase
        we <= 1'b1;
        n  <= n + 1'd1;
        if(n == 9'd3) st <= FINISH;
    end

    PAD: begin
        addr  <= wptr;
        wdata <= 8'hFF;
        we    <= 1'b1;
        wptr  <= wptr + 1'd1;
        if(wptr == TRACK_LEN - 1'd1) st <= FINISH;
    end

    // ---------------- scan ----------------
    // Reads one byte per clock around the track; each byte is handled
    // two clocks after its address is set (v2/o2).

    SCAN: begin
        v1 <= 1'b0;
        if(scnt != SCAN_LEN) begin
            addr <= rptr;
            rptr <= wrap_inc(rptr);
            scnt <= scnt + 1'd1;
            v1   <= 1'b1;
            o1   <= rptr;
        end
        v2 <= v1;
        o2 <= o1;

        if(v2) begin
            sh1 <= sh0;
            sh0 <= rdata;

            if(pst == 2'd1) begin
                // address field: volume, track, sector, checksum
                if(acnt == 3'd7) begin
                    asec <= f_sec[3:0];
                    aok  <= (f_vol ^ f_trk ^ f_sec) == f_chk && f_sec < 8'd16;
                    win  <= 7'd0;
                    pst  <= 2'd2;
                end else
                    ab[acnt] <= rdata;
                acnt <= acnt + 1'd1;
            end else if(sh1 == 8'hD5 && sh0 == 8'hAA && rdata == 8'h96) begin
                // address prologue, also ends an unfinished data search
                acnt <= 3'd0;
                pst  <= 2'd1;
            end else if(pst == 2'd2) begin
                if(sh1 == 8'hD5 && sh0 == 8'hAA && rdata == 8'hAD) begin
                    // first good one wins: it's the same spot after the wrap
                    if(aok && !found[asec]) begin
                        found[asec] <= 1'b1;
                        dofs[asec]  <= wrap_inc(o2);
                    end
                    pst <= 2'd0;
                end else if(win == DATA_WINDOW)
                    pst <= 2'd0;
                win <= win + 1'd1;
            end
        end

        if(scnt == SCAN_LEN && !v1 && !v2) st <= FINISH;
    end

    // ---------------- decode ----------------

    // 86 low-bit values into the scratch area
    D_AUX: begin addr <= wptr; wptr <= wrap_inc(wptr); ret <= D_AUXW; st <= RD_WAIT; end
    D_AUXW: begin
        if(!ug[6]) bad <= 1'b1;
        addr  <= SCRATCH + n;
        wdata <= { 2'd0, ug[5:0] ^ last };
        we    <= 1'b1;
        last  <= ug[5:0] ^ last;
        if(n == 9'd85) begin n <= 9'd0; st <= D_MAIN; end
        else           begin n <= n + 1'd1; st <= D_AUX; end
    end

    // 256 high six bits, each joined with its two low bits
    D_MAIN: begin addr <= wptr; wptr <= wrap_inc(wptr); ret <= D_MAUX; st <= RD_WAIT; end
    D_MAUX: begin
        if(!ug[6]) bad <= 1'b1;
        v6   <= ug[5:0] ^ last;
        last <= ug[5:0] ^ last;
        addr <= SCRATCH + aux_k;
        ret  <= D_MAINW;
        st   <= RD_WAIT;
    end
    D_MAINW: begin
        addr  <= stg | n;
        case(aux_sh)
            2'd0:    wdata <= { v6, sw({ 6'd0, rdata[1:0] }) };
            2'd1:    wdata <= { v6, sw({ 6'd0, rdata[3:2] }) };
            default: wdata <= { v6, sw({ 6'd0, rdata[5:4] }) };
        endcase
        we    <= 1'b1;
        if(n == 9'd255)
            st <= D_CK;
        else begin
            n  <= n + 1'd1;
            st <= D_MAIN;
        end
    end

    // checksum byte: its value must equal the running checksum
    D_CK: begin addr <= wptr; ret <= FINISH; st <= RD_WAIT; n <= 9'd256; end

    FINISH: begin
        if(c_cmd == CMD_DECODE && n == 9'd256)
            ok <= !bad && ug[6] && ug[5:0] == last;
        busy <= 1'b0;
        done <= 1'b1;
        st   <= IDLE;
    end

    default: st <= IDLE;
    endcase
end

endmodule
