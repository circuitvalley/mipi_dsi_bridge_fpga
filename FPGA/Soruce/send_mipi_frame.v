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

module DPHY_TX_FRAME(
					reset_i,				//reset active high
					command_i,
					write_cmd_i,
					clockp_fast_data_i,		//fast clokc input for mipi data
					clocks_fast_clk_i,		//fast clock input for mipi clok
					clock_slow_sync_i,		//slow clock for sync
					lock_chk_i,				//pll lock check
					fifo_almost_empty,
					fifo_empty,
					data_i,					//mipi data input Through FIFO
					fifo_read_en,	
					buf_clkout_lp_p_o,		//mipi clock lp0 out
					buf_clkout_lp_n_o,		//mipi clock lp1 out
					buf_dout_lp_p_o,  		//data lp0 out
					buf_dout_lp_n_o,		//data lp1 out
					byte_clock_o,			//byte clock out
					hs_data_o,			//mipi data out
					hs_clock_o,			//mipi clock out	
					finish_o			// state machine finished transmitting
					);

input wire write_cmd_i;
input wire [7:0]command_i;
input wire reset_i;				//reset active high
input wire clockp_fast_data_i;		//fast clokc input for mipi data
input wire clocks_fast_clk_i;		//fast clock input for mipi clok
input wire clock_slow_sync_i;		//slow clock for sync
input wire fifo_empty;
input wire fifo_almost_empty;
input wire	lock_chk_i;				//pll lock check
input wire [7:0]data_i;					//mipi data input
output reg fifo_read_en;
output wire buf_clkout_lp_p_o;		//mipi clock lp_p out
output wire buf_clkout_lp_n_o;		//mipi clock lp_n out
output wire buf_dout_lp_p_o;  		//data lp_p out
output wire buf_dout_lp_n_o;		//data lp_n out
output wire byte_clock_o;			//byte clock out
output wire hs_data_o;			//mipi data out
output wire hs_clock_o;			//mipi clock out
output reg finish_o;

wire ddr_ready_w;
reg [7:0]command_r;
reg [9:0]rom_addr_r;
wire [7:0]rom_out;
reg rom_read_en;

reg [8:0]rom_tp_addr;
reg rom_tb_read_en; 
ROM rom_inst1 (.Address(rom_addr_r), .OutClock(byte_clock_o), .OutClockEn(rom_read_en), .Reset(reset_i), .Q(rom_out));

reg data_from_rom_nfifo;
reg data_zero;
reg hs_clock_tristate_r;
reg hs_data_tristate_r;

wire write_to_fifo_w;


wire [7:0]mipi_data;
reg [7:0]mipi_data_temp;

reg clkout_lp_p_w;
reg clkout_lp_n_w;
reg dout_lp_p_w;
reg dout_lp_n_w;

DDR_MIPI ddr_mipi_inst1 (	.reset_i(reset_i), 
							.clkout_lp0_i(clkout_lp_p_w),
							.clkout_lp1_i(clkout_lp_n_w), 
							.dout_lp0_i(dout_lp_p_w), 
							.dout_lp1_i(dout_lp_n_w), 
							.clock_slow_i(clock_slow_sync_i), 
							.data_i(mipi_data), 
							.clockp_fast_data_i(clockp_fast_data_i), 
							.clocks_fast_clk_i(clocks_fast_clk_i), 
							.lock_chk_i(lock_chk_i), 
							.tristate_data_i(hs_data_tristate_r),
							.tristate_clk_i(hs_clock_tristate_r),
							
							.byte_clock_o(byte_clock_o), 
							.tx_ready_o(ddr_ready_w), 
							.buf_clkout_lp0_o(buf_clkout_lp_p_o), 
							.buf_clkout_lp1_o(buf_clkout_lp_n_o), 
							.buf_dout_lp0_o(buf_dout_lp_p_o), 
							.buf_dout_lp1_o(buf_dout_lp_n_o),
							.mipi_data_o(hs_data_o),
							.mipi_clock_o(hs_clock_o));



parameter send_frame_state_idle = 4'h0;
parameter send_frame_state_enter_hs_clk1 = 4'h1;
parameter send_frame_state_enter_hs_clk1_wait = 4'h2;
parameter send_frame_state_enter_hs1 = 4'h3;
parameter send_frame_state_enter_hs1_wait = 4'h4;
parameter send_frame_state_enter_hs2 = 4'h5;
parameter send_frame_state_lp_wait1 = 4'h6;
parameter send_frame_state_lp_wait2 = 4'h7;
parameter send_frame_state_enable_hs = 4'h8;
parameter send_frame_state_send_preamble = 4'h9;
parameter send_frame_state_send_rom = 4'hA;
parameter send_frame_state_send_fifo = 4'hB;
parameter send_frame_state_send_crc = 4'hC;
parameter send_frame_state_finish = 4'hD;

reg [4:0]frame_state;
reg [9:0]rom_count;
reg [8:0]zeros_count;
reg [8:0]fifo_count;
reg fifo_empty_reg;
reg data_zero_reg;
reg data_from_rom_nfifo_reg;
			

always @(posedge reset_i or posedge byte_clock_o)	//ddr module samples on falling edge of byte clock , so to align data_zero with posedge
begin
	if (reset_i)
		begin
			data_from_rom_nfifo_reg <= 1'h0;
			data_zero_reg <= 1'h0;
		end
		else
		begin
			
			data_zero_reg <= data_zero;
			data_from_rom_nfifo_reg <= data_from_rom_nfifo;
		end 
end 

assign mipi_data = data_zero_reg? 8'h0:(data_from_rom_nfifo_reg ? rom_out:data_i); //switch between rom or data in or zero


always @(posedge reset_i or negedge byte_clock_o)
begin
	if (reset_i)
		begin
			frame_state <= send_frame_state_idle;
			clkout_lp_p_w <= 1'b1;
			clkout_lp_n_w <= 1'b1;
			dout_lp_p_w <= 1'b1;
			dout_lp_n_w <= 1'b1;
			data_zero <= 1'b1;
			hs_data_tristate_r <= 1'b1; 		//tristate hs data
			hs_clock_tristate_r <= 1'b1; 		//tristate hs clock
			fifo_count <= 9'h0;
			fifo_read_en <= 1'b0;
			finish_o <= 1'b0;
			data_from_rom_nfifo <= 1'b0;
		end
		else
		begin
		case (frame_state)
		 send_frame_state_idle:	begin  	//set dp low , disable clock tri state to un acitve
					clkout_lp_p_w <= 1'b1;	
					clkout_lp_n_w <= 1'b1;
					dout_lp_p_w <= 1'b1;
					dout_lp_n_w <= 1'b1;		
					
					hs_data_tristate_r <= 1'b1; 		//tristate hs data
					hs_clock_tristate_r <= 1'b1; 		//tristate hs clock 
					rom_read_en <= 1'b0;
					
					if (write_cmd_i)
						begin
							finish_o <= 1'b0;
							data_from_rom_nfifo <= 1'b1; //read from rom
							data_zero <= 1'b1;
							rom_addr_r <= command_i; //First address of rom
							frame_state <= send_frame_state_enter_hs_clk1;
						end
						else
						begin
							finish_o <= 1'b1;
						end
				end
		send_frame_state_enter_hs_clk1:begin		//genrate enter hs on clk // first falling edege of byte_clk
					rom_read_en <= 1'b1;
					clkout_lp_p_w <= 1'b0;			//pull clock_lp low
					clkout_lp_n_w <= 1'b1;		
					frame_state <= send_frame_state_enter_hs_clk1_wait;
				end
		send_frame_state_enter_hs_clk1_wait:begin		//to achive 50ns minimum delay between lp and ln
					rom_count[7:0] <= rom_out[7:0];
					rom_addr_r <= rom_addr_r + 1'b1; 			//increse rom address for HSByte of rom count
					
					frame_state <= send_frame_state_enter_hs1;
			end
	    send_frame_state_enter_hs1: begin			//Third falling edege of byte_clk 
					rom_count[9:8] <= rom_out[1:0];	//fetch MSB rom count
					rom_addr_r <= rom_addr_r + 1'b1; 			//increse rom address 
					
					dout_lp_p_w <= 1'b0;
					dout_lp_n_w <= 1'b1;

					
					clkout_lp_p_w <= 1'b0;			//pull both clock_lp low
					clkout_lp_n_w <= 1'b0;		

					hs_clock_tristate_r <= 1'b0; 		//tristate disabled hs clock

					frame_state <= send_frame_state_enter_hs1_wait;
				end
		send_frame_state_enter_hs1_wait: begin		//Fourth Falling edge of byte_clk
					fifo_count[7:0] <= rom_out[7:0];	//fetch expected LSB fifo length
					rom_addr_r <= rom_addr_r + 1'b1; 			//increse rom address 

					frame_state <= send_frame_state_enter_hs2;
				end
	    send_frame_state_enter_hs2: begin			//Fifth falling edege of byte_clk
					fifo_count[8] <= rom_out[0];	//fetch expected MSB fifo length
					dout_lp_p_w <= 1'b0;	
					dout_lp_n_w <= 1'b0;
					
					frame_state <= send_frame_state_lp_wait1;
				end
				
	    send_frame_state_lp_wait1: begin			//6th falling edege of byte_clk, now enable data input mux tristate to be zero start send few null 4 byte
					frame_state <= send_frame_state_lp_wait2;
				end
		
		send_frame_state_lp_wait2: begin			//7th falling edege of byte_clk, now enable data input mux tristate to be zero start send few null 4 byte
					frame_state <= send_frame_state_enable_hs;
				end
	    send_frame_state_enable_hs: begin			//8th- 12th falling edge 
					hs_data_tristate_r <= 1'b0; 		//tristate hs data
					hs_clock_tristate_r <= 1'b0; 		//tristate hs clock 
					frame_state <= send_frame_state_send_preamble;
					zeros_count <= 8'h4;
				end
		send_frame_state_send_preamble:begin		//send zeros
					zeros_count <= zeros_count - 1'h1;
					if (!zeros_count)
						begin
							frame_state <= send_frame_state_send_rom;
							data_zero <= 1'b0;	//ROM samples addr on rising edge , by the time we get to next state correct data will be avilable			
							rom_addr_r <= rom_addr_r + 1'b1; 			//increse rom address
							rom_count <= rom_count - 1'b1;
						end
			    end
	    send_frame_state_send_rom: begin				
					rom_count <= rom_count - 1'b1;
					rom_addr_r <= rom_addr_r + 1'b1; 			//increse rom address
					if (!rom_count)
						begin
						 if (fifo_count )
							begin
									data_from_rom_nfifo <= 1'b0;
									fifo_count <= fifo_count - 1'h1; 
									//fifo_read_en <= !fifo_empty; //enable fifo right now sothat data becomes available by next falling edge of byte clock , if fifo some how empty , keep it disable
									frame_state <= send_frame_state_send_fifo;
								end
							else
								begin
									data_zero <= 1'b1;	
									zeros_count <= 8'h5;	//2 byte of crc (1byte transmits on state transition ) + 4 byte 0
									frame_state <= send_frame_state_send_crc;
								end
						end
						else
						begin
							if (rom_count == 9'h1 && fifo_count) // because need to get rid of first command byte off fifo
							 begin
								 fifo_read_en <= !fifo_empty; //enable fifo right now sothat data becomes available by next falling edge of byte clock , if fifo some how empty , keep it disable
							 end
						end
				end				

	     send_frame_state_send_fifo: begin		//wait till trans finish start sending fifo  if not empty or skip this state on cmmand
				
					fifo_count <= fifo_count - 1'h1; 
					
					//fifo_empty_reg <= fifo_empty; //to get last byte out of fifo
					 
					if (!fifo_count || fifo_empty)		
						begin
							
							fifo_read_en <= 1'b0;
							data_zero <= 1'b1;
							zeros_count <= 4'h4 + fifo_count;	//if any fifo_count is pending but fifo gone empty , pad it with zeros 2 byte of crc (1byte transmits on state transition ) + 4 byte 0 
							frame_state <= send_frame_state_send_crc;
						end
				end
				
		send_frame_state_send_crc: begin				//if 00 00 ff nesessry then use rom
				zeros_count <= zeros_count - 1'h1;
				if (!zeros_count)
					begin
						frame_state <= send_frame_state_finish;
					end
			end
		send_frame_state_finish: begin
				if (!write_cmd_i)
					begin
						frame_state <= send_frame_state_idle;
					end
			end
		default: begin
				frame_state <= send_frame_state_idle;
			end
			
		endcase
		
	  end
end

endmodule