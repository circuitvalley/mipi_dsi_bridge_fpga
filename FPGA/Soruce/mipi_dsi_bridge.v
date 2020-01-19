/* 
 * File:   MIPI_DSI_BRIDGE.v
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

module mipi_dsi_bridge(nsys_reset, clk, 
				hs_clock_o, 
				hs_data_o , 
				buf_clkout_lp_n_o, 
				buf_clkout_lp_p_o, 
				buf_dout_lp_n_o, 
				buf_dout_lp_p_o,
				reg_1v8_en, 
				reg_3v0_en, 
				lcd_rst, 
				bl_en, 
				spi_miso_o, 
				spi_mosi_i, 
				spi_csn_i, 
				spi_clk_i, 
				lcd_test_i);

output reg reg_1v8_en;
output reg reg_3v0_en;
output reg lcd_rst;
output reg bl_en;
output spi_miso_o;
output hs_clock_o;
output hs_data_o;
output wire buf_clkout_lp_n_o ;
output wire buf_clkout_lp_p_o ;
output wire buf_dout_lp_n_o ;
output wire buf_dout_lp_p_o ;

input wire nsys_reset;
input clk; //clock input
input spi_mosi_i;
input spi_csn_i;
input spi_clk_i;
input lcd_test_i;




reg spi_csn_reg1;
reg spi_csn_reg2;
reg spi_csn_reg3;
reg write_cmd;
reg fifo_reset;
reg [7:0]command_r;
reg [2:0] controller_state;
reg [3:0] reset_state;
reg [7:0] spi_cmd;
reg [7:0] line_counter;
reg [32:0] 	delay_counter; 

wire [7:0]fifo_out;
wire pll_clk_spi;
wire reset_i;
wire pll_clk_p;
wire pll_clk_s;
wire pll_clk_s2;
wire ready;
wire sclk;
wire lock;
wire fifo_full;
wire fifo_empty_w;
wire fifo_almost_full;
wire almost_empty;
wire fifo_read_en_w;
wire spi_byte_clock;
wire fifo_write_en_r;
wire [7:0]spi_rxdata;
wire tx_finish_w;


assign  reset_i = !nsys_reset;

assign pll_clk_spi  = pll_clk_s2;

PLL pll_i1 (.CLKI(clk ), .CLKOP(pll_clk_p), .CLKOS(pll_clk_s), .CLKOS2(pll_clk_s2));

FIFo fifo_module( 	.Reset(fifo_reset),
					.RPReset(1'b0),
					.Data(spi_rxdata),
					.Q(fifo_out),
					.WrEn(fifo_write_en_r),
					.RdEn(fifo_read_en_w),
					.WrClock(spi_byte_clock),
					.RdClock(byte_clock_o),
					.Full(fifo_full),
					.AlmostFull(fifo_almost_full), 
					.AlmostEmpty(almost_empty),
					.Empty(fifo_empty_w)
					);
					
DPHY_TX_FRAME dphy_tx_frame(
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
						.hs_clock_o(hs_clock_o));					//mipi clock out	



SPI_SLAVE spi_slave_inst1 ( .clk_i(pll_clk_spi),
							.rst_i(reset_i),
							.rx_data_o(spi_rxdata),
							.byte_clock_o(spi_byte_clock),
							.spi_clk_i(spi_clk_i),
							.spi_mosi_i(spi_mosi_i),
							.spi_csn_i(spi_csn_i)
							);						



parameter controller_state_reset_lcd 		= 3'h0;
parameter controller_state_idle 			= 3'h1;
parameter controller_state_cmd_received  	= 3'h2;
parameter controller_state_cmd_busy 		= 3'h3;
parameter controller_state_wait_cs  		= 3'h4;


parameter state_lcd_activate_vee		= 4'h0;
parameter state_lcd_reset_active		= 4'h1;
parameter state_lcd_reset_wait			= 4'h2;
parameter state_lcd_reset_wait_for_vdd	= 4'h3;
parameter state_lcd_reset_send_cmd_init	= 4'h4;
parameter state_lcd_reset_wait_cmd 		= 4'h5;
parameter state_lcd_reset_wait_init		= 4'h6;
parameter state_lcd_reset_tp_write		= 4'h7;
parameter state_lcd_reset_tp_wait		= 4'h8;


//reset sequence delay timing as per mipi specs
parameter delay_reset_deactivated 	=  32'h1071B00; //250ms @ 46Mhz
parameter delay_reset_ativated  	= 32'h6A980;		//5ms @ 12Mhz
parameter delay_reset_wait_vdd		= 32'h10B80;		//1ms
parameter delay_wait_for_vdd		= 32'h6A980;		//5ms
parameter delay_wait_init			= 32'h6A980;		//5ms


parameter CMD_LCD_INIT = 8'h89; 		//from ROM 13th line
parameter CMD_LCD_TP_FIRST = 8'hCF; 	//line 17
parameter CMD_LCD_TP_NEXT = 8'hD9;		//line 18
parameter DISPLAY_LINE_MAX = 8'd240;

always @(posedge reset_i or posedge byte_clock_o) //must be slower than ddr_byte_clock
begin

if (reset_i)
	begin
		 controller_state <= controller_state_reset_lcd;
		 reset_state <= state_lcd_activate_vee;
		 lcd_rst    <= 1'h0; //reset active
		 reg_1v8_en <= 1'h0;	//reg inactive
		 reg_3v0_en <= 1'h0;	//reg inactive 
		 line_counter <= 8'h0;
	end
	else
	begin
		
		case (controller_state)
			controller_state_reset_lcd: begin
				if (delay_counter)
				begin
					delay_counter <= delay_counter - 1'b1;
				end
				else
				begin
				case (reset_state)
						state_lcd_activate_vee:begin
							reg_1v8_en <= 1'h1; //activate 1v8
							lcd_rst    <= 1'h1; //reset inactive
							bl_en <= 1'h1; //enable BL
							fifo_reset <= 1;
							write_cmd <= 1'b0;
							
							delay_counter <= delay_reset_deactivated;
							reset_state <= state_lcd_reset_active;
							end
							
						state_lcd_reset_active:begin
							fifo_reset <= 1'b0;
							lcd_rst    <= 1'h0; //reset active
							delay_counter <= delay_reset_ativated;
							reset_state <= state_lcd_reset_wait;
							end
							
						state_lcd_reset_wait:begin
							fifo_reset <= 1'b1;
							lcd_rst    <= 1'h1; //reset inactive
							delay_counter <= delay_reset_wait_vdd;
							reset_state <= state_lcd_reset_wait_for_vdd;
							end
							
						state_lcd_reset_wait_for_vdd:begin
							fifo_reset <= 1'b0;
							reg_3v0_en <= 1'h1;	//reg acitve
							delay_counter <= delay_wait_for_vdd;
							reset_state <= state_lcd_reset_send_cmd_init;
							end
							
						state_lcd_reset_send_cmd_init:begin
							write_cmd <= 1'b1; 
							command_r <= CMD_LCD_INIT;
							delay_counter <= 0;
							reset_state <= state_lcd_reset_wait_cmd;
							end
							
						state_lcd_reset_wait_cmd:begin		//lcd take quite some time to process some commands
							if(tx_finish_w)
								begin
									delay_counter <= delay_wait_init;
									reset_state <= state_lcd_reset_wait_init;
								end
								else
								begin
									write_cmd <= 1'b0; 
								end
							end
							
						state_lcd_reset_wait_init:begin
								reset_state <= state_lcd_activate_vee;	//reset, reset state machine
								
								if (lcd_test_i)
								begin
									reset_state <= state_lcd_reset_tp_write;	
									line_counter <= 8'h0;
								end
								else
								begin
									controller_state <= controller_state_idle;
								end
							end
							
						state_lcd_reset_tp_write:begin
							write_cmd <= 1'b1;
							line_counter <= line_counter + 1'b1;
							if (line_counter == 8'h0)
								begin
									command_r <= CMD_LCD_TP_FIRST;
								end
								else
								begin
									command_r <= CMD_LCD_TP_NEXT;
								end
							
							delay_counter <= 0;
							reset_state <= state_lcd_reset_tp_wait;
							end
							
						state_lcd_reset_tp_wait:begin
							if (tx_finish_w)
								begin
									if (line_counter < DISPLAY_LINE_MAX)
									begin
										reset_state <= state_lcd_reset_tp_write;
									end
									else
									begin
										reset_state <= state_lcd_activate_vee;	//reset reset_state
										controller_state <= controller_state_wait_cs;
									end
								end
								else
								begin
									write_cmd <= 1'b0; 
								end

							end
							
						default:begin
								reset_state <= state_lcd_activate_vee;
							end
					endcase

				end
			end
			controller_state_idle: begin
					spi_csn_reg1 <= spi_csn_i;
					spi_csn_reg2 <= spi_csn_reg1;
					spi_csn_reg3 <= spi_csn_reg2;
					if (!spi_csn_reg3 & spi_csn_reg2)
					begin
						write_cmd <= spi_csn_i; 
						command_r <= spi_cmd;
						controller_state <= controller_state_cmd_received;				
					end
				end
			controller_state_cmd_received: begin					
					controller_state <= controller_state_cmd_busy;				
				end
			controller_state_cmd_busy: begin
				if(tx_finish_w)
				begin
					controller_state <= controller_state_wait_cs;
				end
				else
				begin
					write_cmd <= 1'b0; 
				end
				end
			controller_state_wait_cs: begin
				if (!spi_csn_i) //wait till spi_csn_i goes idle
				begin
					controller_state <= controller_state_idle;
				end
				end
			default: begin
				controller_state <= controller_state_idle;
				end 
		endcase
	end
	
end

reg first_byte;

assign fifo_write_en_r = !spi_csn_i;
always @(posedge reset_i or posedge spi_byte_clock or posedge spi_csn_i)
begin
	
	if (reset_i)
	begin
		first_byte <= 1'b0;
		spi_cmd <= 8'b0;
	end
	else 
	begin
		if (spi_csn_i)
		begin
				first_byte <= 1'b0;
		end
		else
		begin
			
			if (first_byte == 1'b0)
			begin
					first_byte <= 1'b1;
					spi_cmd <= spi_rxdata;
			end
		end
	end
end


endmodule
