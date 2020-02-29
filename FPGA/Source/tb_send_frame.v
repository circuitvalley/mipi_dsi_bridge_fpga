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

`timescale 1 ns / 1 ns

module testbenc;

parameter CLK_PERIOD = 1000;
	
	wire reset_g;
	reg reset_i;
	reg [7:0] command_r;
	reg write_cmd;
	reg pll_clk_p;
	reg pll_clk_s;
	reg pll_clk_s2;
	reg almost_empty;
	reg fifo_empty_w;
	reg [7:0]fifo_out;
	
	wire tx_finish_w;
	wire fifo_read_en_w;
	wire buf_clkout_lp_p_o;
	wire buf_clkout_lp_n_o;
	wire buf_dout_lp_p_o;
	wire buf_dout_lp_n_o;	
	wire byte_clock_o;
	wire hs_data_o;	
	wire hs_clock_o;
	wire [7:0]debug1;
	wire [7:0]debug2;

parameter CMD_LCD_INIT = 8'h89; //from ROM 13th line
parameter CMD_LCD_TP_FIRST = 8'hCF; //line 17
parameter CMD_LCD_WRITE_LINE_FIRST = 8'h3F; //line 
parameter CMD_LCD_WRITE_LINE_NEXT = 8'h6B;
parameter CMD_LCD_TP_NEXT = 8'hD9;	//line 18
parameter DISPLAY_LINE_MAX = 8'd240;

GSR GSR_INST (.GSR (reset_g));
PUR PUR_INST (.PUR (reset_g));

DPHY_TX_BYTE dphy_tx_byte(
						.reset_i(reset_i),							//reset active high
						.command_i(command_r),
						.write_cmd_i(write_cmd),
						.clockp_fast_data_i(pll_clk_p),				//fast clokc input for mipi data
						.clocks_fast_clk_i(pll_clk_s),				//fast clock input for mipi clok
						.clock_slow_sync_i(pll_clk_s2),					//slow clock for sync
						.lock_chk_i(1'b1),							//pll lock check
						.fifo_almost_empty(almost_empty),
						.fifo_empty(fifo_empty_w),
						.data_i(fifo_out),							//mipi data input from FIFO			
						
						.finish_o(tx_finish_w),
						.fifo_read_en(fifo_read_en_w),
						.buf_clkout_lp_p_o(buf_clkout_lp_p_o),		//mipi clock lp0 out
						.buf_clkout_lp_n_o(buf_clkout_lp_n_o),		//mipi clock lp1 out
						.buf_dout_lp_p_o(buf_dout_lp_p_o),  		//data lp0 out
						.buf_dout_lp_n_o(buf_dout_lp_n_o),			//data lp1 out
						.byte_clock_o(byte_clock_o),				//byte clock out
						.hs_data_o(hs_data_o),						//mipi data out
						.hs_clock_o(hs_clock_o),						//mipi clock out	
						.debug_out(debug1),//debug_out),
						.debug_adr(debug2));//debug_adr));
					
					
					
initial begin										//genrate 90 phase clock and slow sync clock
	pll_clk_p = 1'b0;
	pll_clk_s = 1'b0;
	pll_clk_s2 = 1'b0;
end
always begin
	#(CLK_PERIOD/2.0) pll_clk_p =  ~pll_clk_p;

end
always begin
	#(CLK_PERIOD/4.0)
	forever #(CLK_PERIOD/2.0) pll_clk_s =  ~pll_clk_s;
end 
always begin
	#(CLK_PERIOD*4.0) pll_clk_s2 =  ~pll_clk_s2;
end 

reg [7:0]line_counter;
initial begin
	reset_i = 1'b0;
	write_cmd = 1'b0;
	almost_empty = 1'b0;
	fifo_empty_w = 1'b0;
	fifo_out = 8'h8C;
	line_counter = DISPLAY_LINE_MAX;
	command_r = CMD_LCD_INIT;
	#50000
	write_cmd = 1'b1;
	#5000
	write_cmd = 1'b0;
	#500000
	while (tx_finish_w == 1'b0)
		begin
	end

	command_r = CMD_LCD_WRITE_LINE_FIRST;
	#50000
	write_cmd = 1'b1;
	#5000
	write_cmd = 1'b0;
	#5000000
	while (tx_finish_w == 1'b0)
		begin
	end
	
	while (line_counter)
	begin
	command_r = CMD_LCD_WRITE_LINE_NEXT; 
	#50000
	write_cmd = 1'b1;
	#5000
	write_cmd = 1'b0;
	#5000000
	while (tx_finish_w == 1'b0)
	begin
		
	end
	line_counter = line_counter - 1'b1;
	end 
	
	$finish;
end 


endmodule