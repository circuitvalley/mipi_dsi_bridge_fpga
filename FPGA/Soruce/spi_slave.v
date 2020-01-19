/* 
 * File:   spi_slave.v
 * Author:  Gaurav
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

//Data change on falling edge sample on rising edge , Spi clk default low

module SPI_SLAVE (clk_i, rst_i, rx_data_o, byte_clock_o, spi_clk_i, spi_mosi_i, spi_csn_i);

input clk_i;			//system clock
input rst_i;			//reset input	
output reg [7:0]rx_data_o;	//received over MOSI
output reg byte_clock_o;	//byte clock goes high when new data on rx_data_o

input spi_clk_i;		//spi clock
input spi_mosi_i;		//spi mosi (rx)
input spi_csn_i;		//spi cs, active low

reg [7:0]rx_data;
reg spi_byte_clk_r;

reg [7:0] rx_reg_r;		//rx shift reg
reg [2:0] rx_bit_count_r;	//count rx 

reg spi_miso_r;
reg spi_byte_clock_r1;
reg spi_byte_clock_r2;
reg spi_byte_clock_r3;
reg spi_byte_clock_r4;

wire spi_clk_w;

assign spi_clk_w = spi_csn_i? 1'b0: spi_clk_i;

always @(posedge rst_i or posedge clk_i)	//cross over to FPGA clock domain
begin
	if (rst_i)
	begin
		byte_clock_o <= 1'b0;
		spi_byte_clock_r1 <= 1'b0;
		spi_byte_clock_r2 <= 1'b0;
		spi_byte_clock_r3 <= 1'b0;
		spi_byte_clock_r4 <= 1'b0;

	end
	else
	begin
		
		spi_byte_clock_r1 <= spi_byte_clk_r;
		spi_byte_clock_r2 <= spi_byte_clock_r1;
		spi_byte_clock_r3 <= spi_byte_clock_r2;
		spi_byte_clock_r4 <= spi_byte_clock_r3;

		if ( !spi_byte_clock_r4 && spi_byte_clock_r3)
		begin
			byte_clock_o <= 1'b0;
			rx_data_o <= rx_data;
		end
		else
		begin
			byte_clock_o <= 1'b1;
		end
	end
end


always @(posedge rst_i or posedge spi_clk_w or posedge spi_csn_i)
begin
	
	if (rst_i)
	begin
		spi_byte_clk_r <= 1'b0; 
		rx_bit_count_r <= 3'b0;
		rx_data <= 0;
	end
	else
	begin
		
		if (spi_csn_i)
		begin
			rx_bit_count_r <= 3'b0;
		  //spi_byte_clk_r <= 1'b0; 
		end
		else
		begin
			rx_bit_count_r <= rx_bit_count_r + 1'b1;
			rx_reg_r <= {rx_reg_r[6:0], spi_mosi_i}; 
			
			if (rx_bit_count_r == 3'h7)
			begin
				rx_data <= {rx_reg_r[6:0], spi_mosi_i};
				spi_byte_clk_r <= 1'b1;
			end
			else if (rx_bit_count_r == 3'h3)
			begin
				spi_byte_clk_r <= 1'b0;
			end	
		end
	end
end


endmodule