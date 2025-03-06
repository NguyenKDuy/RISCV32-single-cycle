`timescale 1ns / 1ps

module tb;
        reg clk = 1'b0;
        reg reset = 1'b0;
        main UUT (.clk(clk), .reset(reset));
        initial begin
                forever #5 clk <= ~clk;
        end
        initial begin
                #1 reset = 1;
                #7 reset = 0;
                #250 $stop;
        end
        
endmodule
