`timescale 1ns / 1ps

module main(
    input clk, reset
    ); 
    wire [31:0] PC_top, instruction_top, imm_ext_top, ALU_result_top, nextnorPC_top, nextjumpPC_top, nextbranchPC_top;
    wire [31:0] read_data1_top, read_data2_top, read_data_top, mux1_top, mux2_top, mux0_top, immediate2address_top, mux_jump_reg, jump_branch_nor_PC;
    wire [3:0] ALU_ctrl_top;
    wire zero_top;
    wire [1:0] ALUOp_top;
    
    //Program_Counter: Done
    
    Program_Counter PC(
        .clk(clk),
        .reset(reset),
        .counter_in(jump_branch_nor_PC),                                  
        
        .counter_out(PC_top));
    
    //  Counter_Adder: Done  
    Counter_Adder CA (
        .counter_in(PC_top), 
        
        .counter_next(nextnorPC_top));                               
    
    //Instruction Memory: Done
    Instruction_Memory IM(
        .clk(clk),
        .reset(reset),
        .read_address(PC_top),
        
        .instruction(instruction_top));
    //Register: Done    
    Registers R(
        .clk(clk), 
        .reset(reset),  
        .RegWrite(RegWrite_top),                                   				
        .read_rs1(instruction_top[19:15]), 
        .read_rs2(instruction_top[24:20]), 
        .write_r(instruction_top[11:7]),
        .write_data(mux_jump_reg),                                  
        
        .read_data1(read_data1_top), 
        .read_data2(read_data2_top));   
     
    //Immediate_Generator: Done  
    Immediate_Generator IG(
        .Opcode(instruction_top[6:0]),
        .instruction(instruction_top),
        
        .imm_ext(imm_ext_top));
        
    //ALU: Done
    ALU ALU(
        .A(read_data1_top), 
        .B(mux0_top),
        .ALU_ctrl(ALU_ctrl_top),
    
        .ALU_result(ALU_result_top),
        .zero(zero_top));
        
    //Control: Done    
    Control C(
        .instruction(instruction_top[6:0]),
    
        .Branch(Branch_top), 
        .MemRead(MemRead_top), 
        .MemtoReg(MemtoReg_top), 
        .ALUOp(ALUOp_top),
        .MemWrite(MemWrite_top), 
        .ALUSrc(ALUSrc_top), 
        .RegWrite(RegWrite_top),
        .Jump(Jump_top));
    
    //ALU_Control: Done
    ALU_Control AC(
        .funct7(instruction_top[30]),
        .funct3(instruction_top[14:12]),
        .ALUOp(ALUOp_top),
        
        .ALU_ctrl(ALU_ctrl_top));    
    
    //Data_Memory: Done
    Data_Memory DM(
        .clk(clk), 
        .reset(reset),
        .MemRead(MemRead_top), 
        .MemWrite(MemWrite_top),
        .address(ALU_result_top),
        .write_data(read_data2_top),
        
        .read_data(read_data_top));    
     
    //Mul0: Done 
    Multiplexer M0(
        .sel(ALUSrc_top),
        .A(read_data2_top), 
        .B(imm_ext_top),
        
        .Mux(mux0_top));  
    
    //Mul1: Done 
    Multiplexer M1(
        .sel(final_branch_top),
        .A(nextnorPC_top), 
        .B(nextbranchPC_top),
        
        .Mux(mux1_top));  
         
    Multiplexer M2(
        .sel(MemtoReg_top),
        .A(ALU_result_top), 
        .B(read_data_top),
        
        .Mux(mux2_top));   
        
           Multiplexer M3(
        .sel(Jump_top),
        .A(mux2_top), 
        .B(nextnorPC_top),
        
        .Mux(mux_jump_reg));   
   
   
         Multiplexer M4(
        .sel(Jump_top),
        .A(mux1_top), 
        .B(nextbranchPC_top),
        
        .Mux(jump_branch_nor_PC));  
       
    //And Logic: Done
    
    //Deal with bne vs. beq
    assign zero_final = (|instruction_top[14:12])^zero_top;
    
    And_Logic AL(
        .Branch(Branch_top), 
        .zero(zero_final),
        
        .jump(final_branch_top));  
    
    
    //Adder: Done    
    Adder A(
        .currPC(PC_top), 
        .offset(imm_ext_top), 
        
        .nextPC(nextbranchPC_top));
    
endmodule

//TODO: Program Counter 32 bit
module Program_Counter(
    input clk, reset, 
    input [31: 0] counter_in,
    output reg [31: 0] counter_out = 'd0
);
    always @(posedge clk or posedge reset) 
    begin
        if (reset) counter_out <= {32{1'b0}};
        else counter_out <= counter_in;
    end
endmodule

//TODO: Counter Add 32 bit
module Counter_Adder (
    input [31:0] counter_in,
    output [31: 0] counter_next 
    );
    assign counter_next = counter_in + 3'd4;
endmodule 

//TODO: Instruction memory 
module Instruction_Memory (
    input clk,          // Thêm clk vào module
    input reset,
    input [31:0] read_address,
    output [31:0] instruction
    );
    integer i = 0;
    reg [31:0] I_MEM [63:0]; 
    integer cnt = 0;
    
    initial begin
        #10 $readmemh("instruction.hex", I_MEM);
    end
    
    // Ð?ng b? reset v?i clock
    always @(posedge clk)  // Ch? dùng posedge clk
    begin
        if (reset) begin
            for (i = 0; i < 64; i = i + 1) begin
                I_MEM[i] <= {32{1'b0}};
            end
        end
    end
    
    assign instruction = I_MEM[read_address >> 2];
    
    initial begin
        $monitor("time %d: instruction[%h]: %h", $time,(read_address >> 2), instruction);
    end
endmodule

//TODO: Register
module Registers (
    input clk, reset, RegWrite, 
    input [4:0] read_rs1, read_rs2, write_r,
    input [31:0] write_data,
    output [31:0] read_data1, read_data2
    );
    integer i;
    reg [31:0] REGISTER [31:0];
    integer status2 = 0;
    integer cnt1 = 0;
    initial begin
        status2 = $fopen("Register.txt","w");
        #250 $fclose(status2); 
        $writememh("Register_file.hex", REGISTER);
    end
    always @(posedge clk or posedge reset)
        begin
                if (reset) begin
                        for (i = 0; i < 32; i = i + 1) begin
                        REGISTER[i] <= 32'b00;
                        end
                end
                else begin
                        if (RegWrite) begin
                                REGISTER[write_r] <= write_data;
                                $fdisplay(status2,"time: %d, Register[%0d]: %h", $time,write_r, write_data);
                        end          
                end 
        end 
        assign read_data1 = REGISTER[read_rs1];
        assign  read_data2 = REGISTER[read_rs2];   
        
endmodule

//TODO: Immediate Generator
module Immediate_Generator (
    input [6:0] Opcode,
    input [31:0] instruction,
    output reg [31:0] imm_ext
);
    always @(*)
    case(Opcode) 
            7'b0000011: imm_ext = {{20{instruction[31]}},instruction[31:20]};                                                                                                     //I - immediate
            7'b0010011: imm_ext = {{20{instruction[31]}},instruction[31:20]};
            7'b0100011: imm_ext = {{20{instruction[31]}},instruction[31:25],instruction[11:7]};                                                                        //S- immediate
            7'b1100011: imm_ext = {{19{instruction[31]}},instruction[31], instruction[7], instruction[30:25], instruction[11:8],1'b0};                                  //B - immediate - 
            7'b1101111: imm_ext = {{12{instruction[31]}},instruction[19:12],instruction[20], instruction[30:25], instruction[24:21],1'b0};                                  // jal
    endcase
endmodule

//TODO: ALU 
module ALU (
    input [31:0] A, B,
    input [3:0] ALU_ctrl,
    output reg [31:0] ALU_result,
    output reg zero = 'b0
);
    always @(*) 
    begin
        case (ALU_ctrl) 
            4'b0010: begin
                ALU_result = A + B; 
                zero = 0;
            end
            4'b0000: begin
                ALU_result = A & B;
                zero = 0;
            end 
            4'b0001: begin 
                ALU_result = A | B; 
                zero = 0;
            end
            
            4'b0110: begin
                zero = (A - B == 0)? 1 : 0;
                ALU_result = A - B;
            end
            
            4'b0011: begin
                ALU_result = A ^ B;
                zero = 0;
            end
            
            4'b1000: begin
                ALU_result = ($signed(A) < $signed(B)) ? 1 : 0;
                zero = 0;
            end
            
            4'b1001: begin
                ALU_result = A < B ? 1 : 0;
                zero = 0;
            end
            
        endcase
    end
    
endmodule

//TODO: Control all
module Control (
    input [6:0] instruction,
    output reg Branch = 'b0, MemRead = 'b0, MemtoReg = 'b0, 
    output reg [1:0] ALUOp = 'b0,
    output reg MemWrite = 'b0, ALUSrc = 'b0, RegWrite = 'b0,
    output reg Jump = 'b0
);
    always @(*)
    begin
        case (instruction)
        7'b0110011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp, Jump} = 9'b001000100;       //R-type
        7'b0000011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp, Jump} = 9'b111100000;       //Load word
        7'b0100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp, Jump} = 9'b100010000;       //Store word
        7'b1100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp, Jump} = 9'b000001010;       //Branch
        7'b0010011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp, Jump} = 9'b101000100;        //Immediate 
        7'b1101111: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp, Jump} = 9'b001000001;        //JAL
        endcase
    end  
endmodule

//TODO: ALU Control 
module ALU_Control (
    input funct7,
    input [2:0] funct3,
    input [1:0] ALUOp,
    output reg [3:0] ALU_ctrl
);
    always @(*) 
    begin
        case ({ALUOp, funct7, funct3})
            6'b00_0_000: ALU_ctrl = 'b0010;             //Load
            6'b00_0_010: ALU_ctrl = 'b0010;             //store

            
            6'b01_0_000: ALU_ctrl = 'b0110;             //Branch
            6'b10_0_000: ALU_ctrl = 'b0010;             //Addi
            6'b10_1_000: ALU_ctrl = 'b0110;             //Sub
            6'b10_0_111: ALU_ctrl = 'b0000;             //And
            6'b10_0_110: ALU_ctrl = 'b0001;             //Or
            
            6'b10_0_100: ALU_ctrl = 'b0011;             //Xor
            6'b10_0_010: ALU_ctrl = 'b1000;             //SLT
            6'b10_0_011: ALU_ctrl = 'b1001;             //SLTU
            6'b10_0_010: ALU_ctrl = 'b1000;             //SLTI
            6'b10_0_011: ALU_ctrl = 'b1001;             //SLTIU
            6'b10_0_111: ALU_ctrl = 'b0000;             //AndI
            6'b10_0_110: ALU_ctrl = 'b0001;             //Ori
            
            6'b01_0_001: ALU_ctrl = 'b0110;             //bne
            
            
        endcase
    end
endmodule

//TODO: Data memory
module Data_Memory (
    input clk, reset,
    input MemRead, MemWrite,
    input [31:0] address,
    input [31:0] write_data,
    output [31:0] read_data 
    );
    reg [31:0] D_MEM [63:0]; //64 bytes Data memory
    integer i;
    
    
    initial begin
        #4      D_MEM[0] = 'h3;
                  D_MEM[1] = 'h4;
         #246 $writememh("DataMem_file.hex", D_MEM);
    end
    
    always @(posedge clk or posedge reset) 
    begin
        if (reset) begin
            for (i = 0; i < 64; i = i + 1) begin
                D_MEM[i] <= 32'b00;
            end
        end
        else if (MemWrite == 1'b1) begin
            D_MEM[address>>2] <= write_data;
            $display("Address to write to Datamem [%0d]: %h", address>>2,write_data);
        end
    end
    assign read_data = (MemRead) ? D_MEM[address>>2] : 32'b00;     // Không th? b? vào always => 1 tín hi?u b?i 2 clock

endmodule

//TODO: Multiplexer
module Multiplexer (
    input sel,
    input [31:0] A, B,
    output [31:0] Mux
);
    assign Mux = (sel == 1'b0) ? A : B;
endmodule

//And logic for beq to jump 
module And_Logic (
    input Branch, zero,
    output jump
    );  
    assign jump = Branch & zero;  
endmodule

module Adder (
    input [31:0] currPC, offset, 
    output [31:0] nextPC
);
    assign nextPC = currPC + offset;
endmodule
