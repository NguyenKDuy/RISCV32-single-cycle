`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/03/2025 04:41:15 PM
// Design Name: 
// Module Name: write
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module write;
        integer status;
        initial begin
                status = $fopen("test.txt", "w");
                $fdisplay ("Write successfully!");
                $fclose(status);
        end

endmodule
