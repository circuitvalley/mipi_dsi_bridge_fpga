/* 
 * File:   tb_spi_bridge.v
 * Copyright:  Gaurav Singh
 * website: www.circuitvalley.com 
 * Created on Jan 19, 2020, 1:33 AM
 *	This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *	Email: gauravsingh@circuitvalley.com
************************************************************************/

`timescale 1ns / 1ns

module test_bench_spi;

parameter CLK_PERIOD = 2;
parameter SPI_CLK_PERIOD = 2;
	reg clock;
	reg spi_clock;
	reg spi_data;
	reg spi_cs;
reg nsys_reset;
wire hs_clock_o;
wire hs_data_o;
wire buf_clkout_lp_n_o;
wire buf_clkout_lp_p_o;
wire buf_dout_lp_n_o;
wire buf_dout_lp_p_o;
wire byte_clock_o;
wire tx_ready_o;
wire write_to_fifo;
wire read_from_fifo_w;
wire reg_1v8_en;
wire reg_3v0_en;
wire lcd_rst;
wire bl_en;
wire spi_miso_o;

wire reset_g;
GSR GSR_INST (.GSR (reset_g));
PUR PUR_INST (.PUR (reset_g));

mipi_dsi_bridge mipi_dsi_bridge_ins(nsys_reset, clock, hs_clock_o, hs_data_o , buf_clkout_lp_n_o, buf_clkout_lp_p_o, buf_dout_lp_n_o, 
				buf_dout_lp_p_o, byte_clock_o, tx_ready_o, write_to_fifo,read_from_fifo_w, 
				reg_1v8_en, reg_3v0_en, lcd_rst, bl_en, spi_miso_o, spi_data, spi_cs, spi_clock, 1'b1);

initial begin
	clock = 1'b0;
	spi_clock = 1'b0;
	spi_data = 1'b0;
	spi_cs = 1'b1;
end


always begin 
	#(CLK_PERIOD/2) clock = ~clock;
end

task send_byte;
	input [7:0] data;
	reg [4:0] i = 4'b0;
	begin
	
		for(i = 4'b0; i< 4'h8; i = i+1) begin
			spi_data = data[(8'h7-i)];
			#(SPI_CLK_PERIOD/2.0)
			spi_clock = 1'b1;
			#(SPI_CLK_PERIOD/2.0)
			spi_clock = 1'b0;
		end 
	end
endtask

task send_line;
	input [7:0]command;
	input data_in;
	reg [7:0]data;
	reg [8:0]i;
	begin
		spi_cs = 1'b0;
		#100
		data = data_in;
		send_byte(command);
		for(i= 9'b0; i < 480; i = i + 1'b1) begin
			#2
			send_byte(data);
			data=data+1'b1;
		end
		#100
		spi_cs = 1'b1;
	end
endtask

task send_line_short; //Error Condition
	input [7:0]command;
	input [7:0]data_in;
	reg [7:0]data;
	reg [8:0]i;
	begin
		spi_cs = 1'b0;
		data = data_in;
		send_byte(command);
		for(i= 9'b0; i < 30; i = i + 1'b1) begin
			#2
			send_byte(data);
			data=data+1'b1;
		end
		#100
		spi_cs = 1'b1;
	end
endtask


task send_frame;
	input [7:0]dummy;
	reg  [8:0]i;
	begin
		send_line(8'h3F, dummy);
		for ( i= 9'h0; i<239; i = i + 1'h1)begin
			#1000
			send_line_short(8'h6B, dummy);
		end
			

		
	end
endtask

initial
begin
	nsys_reset = 1'b0;
	#20
	nsys_reset = 1'b1; 
	#125000
	send_frame(8'h00);
	#5000

	$finish;
end 
endmodule
