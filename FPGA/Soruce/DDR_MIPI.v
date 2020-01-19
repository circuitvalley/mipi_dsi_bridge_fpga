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

`timescale 1 ns / 1 ps
module DDR_MIPI (clkout_lp0_i, buf_clkout_lp0_o,
				clkout_lp1_i, buf_clkout_lp1_o, 
				clock_slow_i, clockp_fast_data_i, 
				clocks_fast_clk_i, mipi_clock_o, 
				lock_chk_i, reset_i, byte_clock_o, 
				tristate_data_i,tristate_clk_i, tx_ready_o, 
				buf_dout_lp0_o, dout_lp0_i, 
				buf_dout_lp1_o, dout_lp1_i, 
				data_i, mipi_data_o)
	/* synthesis NGD_DRC_MASK=1 */;
    
	input wire clkout_lp0_i;			//lp link for mipi clkp
    input wire clkout_lp1_i;			//lp line for mipi clkn
    input wire clock_slow_i;			// slow clock for sync
    input wire clockp_fast_data_i;		//fast clokc input for mipi data
    input wire clocks_fast_clk_i;		//fast clock input for mipi clok
    input wire lock_chk_i;				//clock check
    input wire reset_i;					//reset active high
    input wire tristate_data_i;			//controls HS pin tristate active high
	input wire tristate_clk_i;		//controls HS pin tristate active high
    input wire dout_lp0_i;				//lp line mipi datap


    input wire dout_lp1_i;				//lp line mipi datan
    input wire [7:0] data_i;			//mipi data input
    output wire buf_clkout_lp0_o;		//mipi clock lp0 out
    output wire buf_clkout_lp1_o;		//mipi clock lp1 out
    output wire mipi_clock_o;			//mipi clock out	
    output wire byte_clock_o;			//ddr byte clock output
    output wire tx_ready_o;				//After reset,  Indicate completion of reset synchronization
    output wire [0:0] buf_dout_lp0_o;  //data lp0 out
    output wire [0:0] buf_dout_lp1_o;	//data lp1 out
    output wire [0:0] mipi_data_o;		//mipi data out


    wire opensync_0;
    wire opensync_1;
    wire opensync_cken;
    wire opensync_2;
    wire buf_clkout;
    wire scuba_vhi;
    wire d70;
    wire d60;
    wire d50;
    wire d40;
    wire d30;
    wire d20;
    wire d10;
    wire d00;
    wire eclkc;
    wire sclk_t;
    wire cdiv1;
    wire scuba_vlo;
    wire eclkd;
    wire xstop;
    wire xstart;
    wire opensync_3;
    wire tristate_data_o;
    wire tristate_clk_o;
    wire buf_douto0;

    defparam LUT4_1.initval =  16'h0a78 ;
    ROM16X1A LUT4_1 (.AD3(opensync_0), .AD2(opensync_3), .AD1(lock_chk_i), 
        .AD0(scuba_vhi), .DO0(opensync_cken));

    defparam LUT4_0.initval =  16'hfffe ;
    ROM16X1A LUT4_0 (.AD3(opensync_0), .AD2(opensync_1), .AD1(scuba_vlo), 
        .AD0(scuba_vlo), .DO0(xstop));

    FD1P3BX FF_3 (.D(opensync_3), .SP(opensync_cken), .CK(clock_slow_i), .PD(reset_i), 
        .Q(opensync_0))
             /* synthesis GSR="ENABLED" */;

    FD1P3DX FF_2 (.D(opensync_0), .SP(opensync_cken), .CK(clock_slow_i), .CD(reset_i), 
        .Q(opensync_1))
             /* synthesis GSR="ENABLED" */;

    FD1P3DX FF_1 (.D(opensync_1), .SP(opensync_cken), .CK(clock_slow_i), .CD(reset_i), 
        .Q(opensync_2))
             /* synthesis GSR="ENABLED" */;

    FD1P3DX FF_0 (.D(opensync_2), .SP(opensync_cken), .CK(clock_slow_i), .CD(reset_i), 
        .Q(opensync_3))
             /* synthesis GSR="ENABLED" */;

    OB Inst1_OB (.I(clkout_lp1_i), .O(buf_clkout_lp1_o));
    OB Inst2_OB (.I(clkout_lp0_i), .O(buf_clkout_lp0_o));
    OB Inst3_OB (.I(dout_lp1_i), .O(buf_dout_lp1_o));
    OB Inst4_OB (.I(dout_lp0_i), .O(buf_dout_lp0_o));

    OFS1P3DX Inst8_OFS1P3DX (.D(tristate_clk_i), .SP(scuba_vhi), .SCLK(sclk_t), 
        .CD(reset_i), .Q(tristate_clock_o));

    OFS1P3DX Inst9_OFS1P3DX (.D(tristate_data_i), .SP(scuba_vhi), .SCLK(sclk_t), 
        .CD(reset_i), .Q(tristate_data_o));
		
    OBZ Inst7_OBZ (.I(buf_clkout), .T(tristate_clock_o), .O(mipi_clock_o))
             /* synthesis IO_TYPE="MIPI" */;

    VHI scuba_vhi_inst (.Z(scuba_vhi));

    ODDRX4B Inst6_ODDRX4B (.D0(scuba_vhi), .D1(scuba_vlo), .D2(scuba_vhi), 
        .D3(scuba_vlo), .D4(scuba_vhi), .D5(scuba_vlo), .D6(scuba_vhi), 
        .D7(scuba_vlo), .ECLK(eclkc), .SCLK(sclk_t), .RST(reset_i), .Q(buf_clkout));

    ODDRX4B Inst5_ODDRX4B0 (.D0(d00), .D1(d10), .D2(d20), .D3(d30), .D4(d40), 
        .D5(d50), .D6(d60), .D7(d70), .ECLK(eclkd), .SCLK(sclk_t), .RST(reset_i), 
        .Q(buf_douto0));

    ECLKSYNCA Inst4_ECLKSYNCA (.ECLKI(clocks_fast_clk_i), .STOP(xstop), .ECLKO(eclkc));

    VLO scuba_vlo_inst (.Z(scuba_vlo));

    defparam Inst3_CLKDIVC.DIV = "4.0" ;
    CLKDIVC Inst3_CLKDIVC (.RST(reset_i), .CLKI(eclkd), .ALIGNWD(scuba_vlo), 
        .CDIV1(cdiv1), .CDIVX(sclk_t));

    ECLKSYNCA Inst2_ECLKSYNCA (.ECLKI(clockp_fast_data_i), .STOP(xstop), .ECLKO(eclkd));

    OBZ Inst1_OBZ0 (.I(buf_douto0), .T(tristate_data_o), .O(mipi_data_o[0]))
             /* synthesis IO_TYPE="MIPI" */;

    assign byte_clock_o = sclk_t;
    assign d70 = data_i[7];
    assign d60 = data_i[6];
    assign d50 = data_i[5];
    assign d40 = data_i[4];
    assign d30 = data_i[3];
    assign d20 = data_i[2];
    assign d10 = data_i[1];
    assign d00 = data_i[0];
    assign tx_ready_o = xstart;
    assign xstart = opensync_3;


    // exemplar begin
    // exemplar attribute FF_3 GSR ENABLED
    // exemplar attribute FF_2 GSR ENABLED
    // exemplar attribute FF_1 GSR ENABLED
    // exemplar attribute FF_0 GSR ENABLED
    // exemplar attribute Inst12_BB IO_TYPE MIPI_LP
    // exemplar attribute Inst11_BB IO_TYPE MIPI_LP
    // exemplar attribute Inst10_BB0 IO_TYPE MIPI_LP
    // exemplar attribute Inst9_BB0 IO_TYPE MIPI_LP
    // exemplar attribute Inst7_OBZ IO_TYPE MIPI
    // exemplar attribute Inst1_OBZ0 IO_TYPE MIPI
    // exemplar end

endmodule
