//KSHITIJ GUPTA
//2017B3A70601P

module ripple_counter (Q,clk,rst);
output [3:0]Q ;
input clk,rst;
wire [3:0] Qbar;

seff_b ff1(Q[0], Qbar[0],Q[0],Qbar[0],clk,rst);
seff_b ff2(Q[1], Qbar[1],Q[1],Qbar[1],Qbar[0],rst);
seff_b ff3(Q[2], Qbar[2],Q[2],Qbar[2],Qbar[1],rst);
seff_b ff4(Q[3], Qbar[3],Q[3],Qbar[3],Qbar[2],rst);

endmodule

module seff_b(r,s,q,qbar,clk,rst);

input s,r,clk,rst;
output reg q, qbar;

always@(posedge clk)
begin
	if (rst==1)
    	begin
    	q=0;
    	qbar=1;
    	end
    
	else if(s == 1)
    	begin
    	q = 1;
    	qbar = 0;
    	end
   	 
	else if(r == 1)
    	begin
    	q = 0;
    	qbar =1;
    	end
   	 
	else if(s == 0 & r == 0)
	begin
	q <= q;
	qbar <= qbar;
	end
end
endmodule

module parity_checker (in, parity, res);

	input [7:0] in;
	input parity;
    
	output reg res;
    
	wire par_computed;

	xnor(par_computed, in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7]);
    
	always@(*)
	begin
    
    	if (par_computed == parity)
        	res = 1;
    	else
        	res = 0;
	end
endmodule

module MUX2to1(a,b,sel,out);
input a,b;
input sel;
output out;
assign out = a & (~sel) | b & sel;
endmodule


module MUX16To8(out, mb1, mb2, select);
  input [7:0]mb1, mb2;
  input select;
  output  [7:0]out;
 
  MUX2to1  mux1( mb1[0], mb2[0], select,out[0]);
  MUX2to1  mux2( mb1[1], mb2[1], select,out[1]);
 MUX2to1 mux3( mb1[2], mb2[2], select,out[2]);
  MUX2to1  mux4( mb1[3], mb2[3], select,out[3]);
  MUX2to1  mux5( mb1[4], mb2[4], select,out[4]);
  MUX2to1  mux6( mb1[5], mb2[5], select,out[5]);
 MUX2to1 mux7( mb1[6], mb2[6], select,out[6]);
 MUX2to1  mux8(mb1[7], mb2[7], select,out[7]);
 
endmodule

module MEM1(s,memory_out,parity_out);
input [2:0]s;
output reg [7:0]memory_out;
output reg parity_out;
reg [7:0] memory[0:7];
reg parity[0:7];

always @ (s) begin
case(s)
	3'b000 : begin
    	memory_out <= memory[0];
    	parity_out <=parity[0];
    	end
	3'b001 : begin
    	memory_out <= memory[1];
    	parity_out <=parity[1];
    	end
	3'b010 : begin
    	memory_out <= memory[2];
    	parity_out <=parity[2];
    	end
	3'b011 : begin
    	memory_out <= memory[3];
    	parity_out <=parity[3];
    	end
	3'b100 : begin
    	memory_out <= memory[4];
    	parity_out <=parity[4];
    	end
	3'b101 : begin
    	memory_out <= memory[5];
    	parity_out <=parity[5];
    	end
	3'b110 : begin
    	memory_out <= memory[6];
    	parity_out <=parity[6];
    	end
	3'b111 : begin
    	memory_out <= memory[7];
    	parity_out <=parity[7];
    	end
    
	endcase
end
initial begin
memory[0]<=8'b0001_1111;
parity[0]<=1'b1;
memory[1]<=8'b0011_0001;
parity[1]<=1'b1;
memory[2]<=8'b0101_0011;
parity[2]<=1'b1;
memory[3]<=8'b0111_0101;
parity[3]<=1'b1;
memory[4]<=8'b1001_0111;
parity[4]<=1'b1;
memory[5]<=8'b1011_1001;
parity[5]<=1'b1;
memory[6]<=8'b1101_1011;
parity[6]<=1'b1;
memory[7]<=8'b1111_1101;
parity[7]<=1'b1;
end
endmodule

module MEM2(s,memory_out,parity_out);
input [2:0]s;
output reg [7:0]memory_out;
output reg parity_out;
reg [7:0] memory[0:7];
reg parity[0:7];

always @ (s) begin
case(s)
	3'b000 : begin
    	memory_out <= memory[0];
    	parity_out <=parity[0];
    	end
	3'b001 : begin
    	memory_out <= memory[1];
    	parity_out <=parity[1];
    	end
	3'b010 : begin
    	memory_out <= memory[2];
    	parity_out <=parity[2];
    	end
	3'b011 : begin
    	memory_out <= memory[3];
    	parity_out <=parity[3];
    	end
	3'b100 : begin
    	memory_out <= memory[4];
    	parity_out <=parity[4];
    	end
	3'b101 : begin
    	memory_out <= memory[5];
    	parity_out <=parity[5];
    	end
	3'b110 : begin
    	memory_out <= memory[6];
    	parity_out <=parity[6];
    	end
	3'b111 : begin
    	memory_out <= memory[7];
    	parity_out <=parity[7];
    	end
    
	endcase
end
initial begin
memory[0]<=8'b0000_0000;
parity[0]<=1'b0;
memory[1]<=8'b0010_0010;
parity[1]<=1'b0;
memory[2]<=8'b0100_0100;
parity[2]<=1'b0;
memory[3]<=8'b0110_0110;
parity[3]<=1'b0;
memory[4]<=8'b1000_1000;
parity[4]<=1'b0;
memory[5]<=8'b1010_1010;
parity[5]<=1'b0;
memory[6]<=8'b1100_1100;
parity[6]<=1'b0;
memory[7]<=8'b1110_1110;
parity[7]<=1'b0;
end
endmodule




module Fetch_Data(in,fet_mem,fet_parity);
input [3:0]in;
output [7:0]fet_mem;
output fet_parity;

wire [7:0] memory_out1;
wire [7:0] memory_out2;
wire parity_out1,parity_out2;
MEM1 mem1(in[2:0],memory_out1,parity_out1);
MEM2 mem2(in[2:0],memory_out2,parity_out2);

MUX16To8 f1(fet_mem, memory_out1, memory_out2, in[3]);
MUX2to1 f2(parity_out1,parity_out2,in[3],fet_parity);
endmodule

module DESIGN(clk,rstn,Addr,res);
input clk;
input rstn;
wire [3:0]out;
output res;
input [3:0] Addr;
wire v1;
wire [7:0]v2 ;
ripple_counter  f1 (out, clk, rstn);
Fetch_Data f2( Addr, v2, v1);
parity_checker f3(v2, v1, res);
endmodule

module TestBench;
	reg clk,rstn;
	reg [3:0] Addr;
wire res;
	initial
    	clk = 1'b1;

	always
    	#5 clk = ~clk;

DESIGN ds(clk,rstn,Addr,res);

	initial begin
	$monitor($time,"clock = %b, result = %b ",clk,res);
        	rstn <= 1'b1;
    	#10  rstn <= 1'b0;
    	#10 Addr <= 4'b1001;
    	#10 Addr <= 4'b0011;
    	#10 Addr <= 4'b0010;  
    	#10 Addr <= 4'b0101;
    	#10 $finish;
	end
endmodule







