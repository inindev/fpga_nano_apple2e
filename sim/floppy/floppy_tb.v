// Testbench for floppy_track: .dsk/.po load and save, and .nib behaviour.
// Run with run.sh, which makes the reference data and runs every part.
//
//   default    .dsk/.po suite, checked against dsk2nib output
//   NIBTEST    .nib scenario; prints its SD transactions so the old and
//              new floppy_track can be compared (OLD = old module, no format port)

`timescale 1ns/1ps

module floppy_tb;

localparam TRACK_LEN = 6656, DSK_TRACK = 4096, DSK_SIZE = 143360, NIB_SIZE = 232960;

reg clk = 0, clk2 = 0;
always #35   clk  = ~clk;     // Apple, ~14.3 MHz
always #18.5 clk2 = ~clk2;    // system, ~27 MHz

// Gowin's primitive models refer to a global set/reset instance by this name
GSR GSR(.GSRI(1'b1));

reg         reset = 0, change = 0, mount = 0, active = 0;
reg   [1:0] format = 0;
reg   [5:0] track = 0;
reg  [12:0] ram_addr = 0;
reg   [7:0] ram_di = 0;
reg         ram_we = 0;
wire  [7:0] ram_do;
wire        ready, busy;

wire [31:0] sd_lba;
wire        sd_rd, sd_wr;
reg         sd_ack = 0;
reg   [8:0] sd_buff_addr = 0;
reg   [7:0] sd_buff_dout = 0;
wire  [7:0] sd_buff_din;
reg         sd_buff_wr = 0;

floppy_track dut(
    .clk(clk), .clk2(clk2), .reset(reset),
    .sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
    .sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
    .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
    .change(change), .mount(mount),
`ifndef OLD
    .format(format),
`endif
    .track(track), .ready(ready), .active(active),
    .ram_addr(ram_addr), .ram_do(ram_do), .ram_di(ram_di), .ram_we(ram_we),
    .busy(busy)
);

reg [7:0] img      [0:NIB_SIZE-1];   // the mounted file on the SD card
reg [7:0] a_dsk    [0:DSK_SIZE-1];
reg [7:0] b_dsk    [0:DSK_SIZE-1];
reg [7:0] a_nib    [0:NIB_SIZE-1];
reg [7:0] b_nib    [0:NIB_SIZE-1];
reg [7:0] a_po_nib [0:NIB_SIZE-1];

// physical sector -> 256 byte block of the track in the image
function integer dos_order(input integer p);
    case(p)
        0: dos_order = 0;  1: dos_order = 7;  2: dos_order = 14; 3: dos_order = 6;
        4: dos_order = 13; 5: dos_order = 5;  6: dos_order = 12; 7: dos_order = 4;
        8: dos_order = 11; 9: dos_order = 3;  10: dos_order = 10; 11: dos_order = 2;
        12: dos_order = 9; 13: dos_order = 1; 14: dos_order = 8;  default: dos_order = 15;
    endcase
endfunction
function integer po_order(input integer p);
    po_order = (p == 15) ? 15 : (p % 2) ? 8 + p / 2 : p / 2;
endfunction

integer errors = 0;

task fail(input [8*64-1:0] what, input integer at, input integer got, input integer exp);
    begin
        errors = errors + 1;
        if(errors <= 20) $display("FAIL %0s at %0d: got %02h expected %02h", what, at, got, exp);
    end
endtask

// ---------------- SD card model ----------------

always begin : sd_model
    reg wr;
    reg [31:0] l;
    integer i;
    @(posedge clk2);
    if(sd_rd || sd_wr) begin
        wr = sd_wr;
        l  = sd_lba;
`ifdef NIBTEST
        $display("SD %s %0d @%0t", wr ? "W" : "R", l, $time);
`endif
        repeat(5) @(posedge clk2);
        @(negedge clk2) sd_ack = 1;
        for(i = 0; i < 512; i = i + 1) begin
            @(negedge clk2);
            sd_buff_addr = i;
            if(!wr) begin
                sd_buff_dout = img[l * 512 + i];
                sd_buff_wr = 1;
                @(negedge clk2) sd_buff_wr = 0;
            end else begin
                @(negedge clk2);
                @(negedge clk2);
                img[l * 512 + i] = sd_buff_din;
            end
        end
        @(negedge clk2) sd_ack = 0;
    end
end

// ---------------- Apple side ----------------

// one operation started by the stimulus: wait for it to finish
task wait_op;
    begin
        wait(dut.busyi === 1'b1);
        wait(dut.busyi === 1'b0);
        repeat(4) @(posedge clk2);
    end
endtask

task do_mount(input [1:0] f, input [5:0] t);
    begin
        // the head doesn't move during a mount: let the track settle through
        // floppy_track's synchronizer first
        @(negedge clk2);
        track = t;
        repeat(8) @(negedge clk2);
        format = f;
        mount = 1;
        change = 1;
        repeat(4) @(negedge clk2);
        change = 0;
        wait_op;
    end
endtask

task step_to(input [5:0] t);
    begin
        @(negedge clk);
        track = t;
        wait_op;
    end
endtask

// compare the track buffer with 6656 bytes of a reference nib image
task check_buffer(input integer which, input integer t);
    integer i;
    reg [7:0] e;
    begin
        for(i = 0; i <= TRACK_LEN; i = i + 1) begin
            @(negedge clk);
            if(i > 0) begin
                case(which)
                    0: e = a_nib[t * TRACK_LEN + i - 1];
                    1: e = b_nib[t * TRACK_LEN + i - 1];
                    default: e = a_po_nib[t * TRACK_LEN + i - 1];
                endcase
                if(ram_do !== e) fail("track buffer", t * TRACK_LEN + i - 1, ram_do, e);
            end
            if(i < TRACK_LEN) ram_addr = i;
        end
    end
endtask

// the Apple writes byte d at track position a
task apple_wr(input integer a, input [7:0] d);
    begin
        @(negedge clk);
        ram_addr = a % TRACK_LEN;
        ram_di = d;
        ram_we = 1;
        @(negedge clk);
        ram_we = 0;
    end
endtask

// the Apple rewrites the whole track from b_nib, starting at position rot
task write_b_track(input integer t, input integer rot);
    integer i;
    begin
        for(i = 0; i < TRACK_LEN; i = i + 1)
            apple_wr(i + rot, b_nib[t * TRACK_LEN + i]);
    end
endtask

// expected image contents for one 256 byte block
task check_block(input integer t, input integer blk, input integer from_b);
    integer i, o;
    reg [7:0] e;
    begin
        for(i = 0; i < 256; i = i + 1) begin
            o = t * DSK_TRACK + blk * 256 + i;
            e = from_b ? b_dsk[o] : a_dsk[o];
            if(img[o] !== e) fail("image", o, img[o], e);
        end
    end
endtask

// block b of a PO image, expected to hold the data of physical sector p of b_dsk
task check_po_block(input integer t, input integer p);
    integer i, o;
    begin
        for(i = 0; i < 256; i = i + 1) begin
            o = t * DSK_TRACK + po_order(p) * 256 + i;
            if(img[o] !== b_dsk[t * DSK_TRACK + dos_order(p) * 256 + i])
                fail("po image", o, img[o], b_dsk[t * DSK_TRACK + dos_order(p) * 256 + i]);
        end
    end
endtask

task load_img_a_dsk;
    integer i;
    begin
        for(i = 0; i < DSK_SIZE; i = i + 1) img[i] = a_dsk[i];
    end
endtask

// every track other than t still holds image a
task check_others(input integer t);
    integer tt, blk;
    begin
        for(tt = 0; tt < 35; tt = tt + 1)
            if(tt != t)
                for(blk = 0; blk < 16; blk = blk + 1) check_block(tt, blk, 0);
    end
endtask

integer t, p, i, e0;

initial begin
    $readmemh("a_dsk.hex", a_dsk);
    $readmemh("b_dsk.hex", b_dsk);
    $readmemh("a_nib.hex", a_nib);
    $readmemh("b_nib.hex", b_nib);
    $readmemh("a_po_nib.hex", a_po_nib);

`ifdef NIBTEST
    // ---------------- .nib ----------------
    for(i = 0; i < NIB_SIZE; i = i + 1) img[i] = a_nib[i];
    do_mount(2'd0, 6'd0);
    check_buffer(0, 0);
    step_to(6'd3);
    check_buffer(0, 3);
    // write, then idle: save without a load
    active = 1;
    for(i = 0; i < 200; i = i + 1) apple_wr(6600 + i, i);
    active = 0;
    wait_op;
    for(i = 0; i < 200; i = i + 1)
        if(img[3 * TRACK_LEN + (6600 + i) % TRACK_LEN] !== i[7:0])
            fail("nib save", 3 * TRACK_LEN + (6600 + i) % TRACK_LEN, img[3 * TRACK_LEN + (6600 + i) % TRACK_LEN], i);
    // write, then step: save and load
    step_to(6'd10);
    active = 1;
    for(i = 0; i < 50; i = i + 1) apple_wr(i * 7, 8'hA5);
    step_to(6'd11);
    active = 0;
    check_buffer(0, 11);
    for(i = 0; i < 50; i = i + 1)
        if(img[10 * TRACK_LEN + i * 7] !== 8'hA5) fail("nib step save", 10 * TRACK_LEN + i * 7, img[10 * TRACK_LEN + i * 7], 8'hA5);
    begin : img_sum
        integer s;
        s = 0;
        for(i = 0; i < NIB_SIZE; i = i + 1) s = (s * 31 + img[i]) & 32'h7fffffff;
        $display("IMG %08h", s);
    end
`else
    // ---------------- 1: .dsk load, every track ----------------
    load_img_a_dsk;
    do_mount(2'd1, 6'd0);
    check_buffer(0, 0);
    for(t = 1; t < 35; t = t + 1) begin
        step_to(t);
        check_buffer(0, t);
    end
    $display("1 dsk load: %0d errors", errors);

    // ---------------- 2: .po load ----------------
    e0 = errors;
    load_img_a_dsk;
    do_mount(2'd2, 6'd0);
    check_buffer(2, 0);
    for(t = 1; t < 35; t = t + 1) begin
        step_to(t);
        check_buffer(2, t);
    end
    $display("2 po load: %0d errors", errors - e0);

    // ---------------- 3: whole track rewritten across the wrap ----------------
    e0 = errors;
    load_img_a_dsk;
    do_mount(2'd1, 6'd17);
    active = 1;
    write_b_track(17, 6000);
    active = 0;                     // idle: save
    wait_op;
    for(p = 0; p < 16; p = p + 1) check_block(17, p, 1);
    check_others(17);
    $display("3 dsk save, whole track: %0d errors", errors - e0);

    // ---------------- 4: one sector rewritten like DOS does ----------------
    // data field of physical sector 3 from b, written 3 bytes early, over the old one
    e0 = errors;
    load_img_a_dsk;
    do_mount(2'd1, 6'd5);
    active = 1;
    for(i = 0; i < 349; i = i + 1)
        apple_wr(3 * 409 + 57 + i, b_nib[5 * TRACK_LEN + 3 * 409 + 60 + i]);
    active = 0;
    wait_op;
    for(p = 0; p < 16; p = p + 1) check_block(5, p, p == dos_order(3));
    check_others(5);
    $display("4 dsk save, one sector: %0d errors", errors - e0);

    // ---------------- 5: bad checksums ----------------
    // physical 6 bad: its block keeps a, the other half of its SD sector (physical 4) gets b.
    // physical 7 and 5 bad, both halves of SD sector 2: that SD sector isn't written.
    e0 = errors;
    load_img_a_dsk;
    do_mount(2'd1, 6'd9);
    active = 1;
    write_b_track(9, 1234);
    for(p = 5; p <= 7; p = p + 1)
        apple_wr(1234 + p * 409 + 63 + 100,
                 b_nib[9 * TRACK_LEN + p * 409 + 63 + 100] == 8'h96 ? 8'h97 : 8'h96);
    active = 0;
    wait_op;
    for(p = 0; p < 16; p = p + 1)
        check_block(9, dos_order(p), !(p == 5 || p == 6 || p == 7));
    check_others(9);
    $display("5 dsk save, bad sectors: %0d errors", errors - e0);

    // ---------------- 6: dirty track, then step: save and load ----------------
    e0 = errors;
    load_img_a_dsk;
    do_mount(2'd1, 6'd20);
    active = 1;
    write_b_track(20, 333);
    step_to(6'd21);
    active = 0;
    check_buffer(0, 21);
    for(p = 0; p < 16; p = p + 1) check_block(20, p, 1);
    check_others(20);
    $display("6 dsk save on step: %0d errors", errors - e0);

    // ---------------- 7: .po save ----------------
    e0 = errors;
    load_img_a_dsk;
    do_mount(2'd2, 6'd30);
    active = 1;
    write_b_track(30, 4321);
    active = 0;
    wait_op;
    for(p = 0; p < 16; p = p + 1) check_po_block(30, p);
    $display("7 po save: %0d errors", errors - e0);
`endif

    $display("%0d errors: %s", errors, errors ? "FAIL" : "PASS");
    $finish;
end

endmodule
