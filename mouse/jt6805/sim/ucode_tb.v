// Brute-force equivalence test: jt6805_ucode against the Gowin pROM IP it
// replaces (Gowin_pROM_uc, simulated with Gowin's primitive models).
// Every address is read in order, then random addresses change every clock.
// Run with run_ucode_tb.sh.

`timescale 1ns/1ps

module ucode_tb;

reg         clk = 0;
reg  [11:0] ad  = 0;
wire [39:0] ref_dout;
wire [38:0] new_dout;

always #5 clk = ~clk;

// Gowin's primitive models refer to a global set/reset instance by this name
GSR GSR(.GSRI(1'b1));

Gowin_pROM_uc ref_rom(
    .dout(ref_dout), .clk(clk), .oce(1'b1), .ce(1'b1), .reset(1'b0), .ad(ad)
);

jt6805_ucode new_rom(.clk(clk), .ad(ad), .dout(new_dout));

integer i, errors = 0, checks = 0;
// called at negedge, before ad changes: dout holds the data for ad
task check;
    begin
        checks = checks + 1;
        if (ref_dout[39] !== 1'b0 || ref_dout[38:0] !== new_dout) begin
            errors = errors + 1;
            if (errors <= 10)
                $display("MISMATCH ad=%03h ref=%010h new=%010h", ad, ref_dout, new_dout);
        end
    end
endtask

initial begin
    // sequential sweep of every address, plus one to flush the last read
    for (i = 0; i <= 4096; i = i + 1) begin
        @(negedge clk);
        if (i > 0) check;
        ad = i[11:0];
    end
    // random addresses, a new one every clock
    for (i = 0; i < 200000; i = i + 1) begin
        @(negedge clk);
        check;
        ad = $random;
    end
    $display("%0d checks, %0d mismatches: %s", checks, errors, errors ? "FAIL" : "PASS");
    $finish;
end

endmodule
