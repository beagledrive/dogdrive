module top (input         clk,
            output [3:0]  led,
            inout  [15:0] gpmc_ad,     // data register
            input         gpmc_advn,
            input         gpmc_csn1,
            input         gpmc_wein,
            input         gpmc_oen,
            input         gpmc_clk,
	    output	  irq_in,
            input  [1:0]  btn,
            output [7:0]  pmod1,
            output [7:0]  pmod2,
            output [7:0]  pmod3,
            output [7:0]  pmod4,);

parameter ADDR_WIDTH = 4;
parameter DATA_WIDTH = 16;
parameter RAM_DEPTH = 1 << ADDR_WIDTH;

reg [DATA_WIDTH-1:0] mem [0:RAM_DEPTH];

reg oe;
reg we;
reg cs;

// Countermax = 2s with 200M clk
reg [24:0] counter = 0;

wire[ADDR_WIDTH-1:0]  addr;
reg [DATA_WIDTH-1:0]  data_out;
wire [DATA_WIDTH-1:0]  data_in;

// set period to PWM period
//parameter PERIOD = 1/10000;
parameter PERIOD_FPGA = 5;
parameter PERIOD_CLK = 20000; //PERIOD*1000000000/PERIOD_FPGA;  // 10kHz period with 200MHz clock is 20000 cycles
parameter IRQ_PERIOD = 100;  
parameter IRQ_TRIGGER = 0; // IRQ start
parameter DVALID_TRIGGER = 200; // Time before end period when valid data required


wire [31:0] sw_on_a_mem;
wire [31:0] sw_off_a_mem;
wire [31:0] sw_on_b_mem;
wire [31:0] sw_off_b_mem;
wire [31:0] sw_on_c_mem;
wire [31:0] sw_off_c_mem;

wire pwm_en; 
wire pwm_re; // Reset
wire pwm_pol; // polarity
wire dsp_err; // dsp sets to 0 on data transfer, fpga resets to 1 at start of period
wire fpga_err; // dsp reads each cycle and if it is 1 will stop

assign sw_on_a_mem[15:0] = mem[2];
assign sw_on_a_mem[31:16] = mem[3];
assign sw_off_a_mem[15:0] = mem[4];
assign sw_off_a_mem[31:16] = mem[5];
assign sw_on_b_mem[15:0] = mem[6];
assign sw_on_b_mem[31:16] = mem[7];
assign sw_off_b_mem[15:0] = mem[8];
assign sw_off_b_mem[31:16] = mem[9];
assign sw_on_c_mem[15:0] = mem[10];
assign sw_on_c_mem[31:16] = mem[11];
assign sw_off_c_mem[15:0] = mem[12];
assign sw_off_c_mem[31:16] = mem[13];

assign pwm_re = mem[0][0];
assign pwm_en = mem[0][1];
assign pwm_pol = mem[0][2];
assign dsp_err = mem[1][0]; 
assign fpga_err = mem[14][0];

reg [31:0] sw_on_a_next;
reg [31:0] sw_off_a_next;
reg [31:0] sw_on_a_curr;
reg [31:0] sw_off_a_curr;
reg [31:0] sw_on_b_next;
reg [31:0] sw_off_b_next;
reg [31:0] sw_on_b_curr;
reg [31:0] sw_off_b_curr;
reg [31:0] sw_on_c_next;
reg [31:0] sw_off_c_next;
reg [31:0] sw_on_c_curr;
reg [31:0] sw_off_c_curr;

reg dsp_err_reg;
reg fpga_err_reg;
reg pwm_en_reg;


// Main Routine
always @ (posedge clk_200m)
begin
	if (pwm_re)begin
	//	counter <= 0;
//		fpga_err <=0;
	end
	
//	fpga_err <= fpga_err;

	if (1) begin
		// IRQ occurs at start period - could change to before start
		if (counter == IRQ_TRIGGER) begin
			irq_in <= 1;
		end else if (counter == (IRQ_TRIGGER+IRQ_PERIOD)) begin
			irq_in <= 0;
		end
	
		// Complete PWM Phase A switching
		if (counter >= sw_off_a_curr) begin
			pmod1[0] <= 0;
		end else if (counter >= sw_on_a_curr) begin
			pmod1[0] <= 1;
		end else begin
			pmod1[0] <= 0;
		end	

		// Complete PWM Phase B switching
		if (counter >= sw_off_b_curr) begin
			pmod1[1] <= 0;
		end else if (counter >= sw_on_b_curr) begin
			pmod1[1] <= 1;
		end else begin
			pmod1[1] <= 0;
		end	

		// Complete PWM Phase C switching
		if (counter >= sw_off_c_curr) begin
			pmod1[2] <= 0;
		end else if (counter >= sw_on_c_curr) begin
			pmod1[2] <= 1;
		end else begin
			pmod1[2] <= 0;
		end	

		//  Reset data valid register to zero
		if (counter == IRQ_TRIGGER) begin
			mem[1][0] <= 1;
		end



		// Complete data check and protection before end period 
		if ((counter == (PERIOD_CLK - DVALID_TRIGGER))) begin	
			// HOLD - protection functions
			// Currently just set the registers to be the memory location
			// value in sw on mem is a clock counter (ns)
			if (!dsp_err) begin
				sw_on_a_next <= sw_on_a_mem;
				sw_off_a_next <= sw_off_a_mem;
				sw_on_b_next <= sw_on_b_mem;
				sw_off_b_next <= sw_off_b_mem;			
				sw_on_c_next <= sw_on_c_mem;
				sw_off_c_next <= sw_off_c_mem;
			end else begin
				sw_on_a_next <= 0;
				sw_off_a_next <=0;
				sw_on_b_next <= 0;
				sw_off_b_next <=0;
				sw_on_c_next <= 0;
				sw_off_c_next <=0;
				fpga_err_reg <= 1;
//				mem[0][1] <= 0;
			end
		end

		// At end of period set the next switches to the current 
		if (counter == PERIOD_CLK) begin 
			sw_on_a_curr <= sw_on_a_next;
			sw_off_a_curr <= sw_off_a_next;
			sw_on_b_curr <= sw_on_b_next;
			sw_off_b_curr <= sw_off_b_next;
			sw_on_c_curr <= sw_on_c_next;
			sw_off_c_curr <= sw_off_c_next;
		end
	
		// Reset or increment counter
		if (counter == PERIOD_CLK) begin
			counter <= 0;
		end else begin
			counter <= counter + 1'b1;
		end
	end
end

// terminology is from DSP / GPMC perspective
// write enable so set memory address to GPMC pin data
always @ (posedge clk_200m)
begin
	if (!cs && !we && oe) begin
        	mem[addr] <= data_out;
	end 
end

// output enable so set the data in to the memory address
always @ (posedge clk_200m)
begin
	if (!cs && we && !oe) begin
        	data_in <= mem[addr];
	end else begin
        	data_in <= 0;
	end
end

gpmc_sync #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH))
gpmc_controller (
    .clk(clk),

    .gpmc_ad(gpmc_ad),
    .gpmc_advn(gpmc_advn),
    .gpmc_csn1(gpmc_csn1),
    .gpmc_wein(gpmc_wein),
    .gpmc_oen(gpmc_oen),
    .gpmc_clk(gpmc_clk),

    .oe(oe),
    .we(we),
    .cs(cs),
    .address(addr),
    .data_out(data_out),
    .data_in(data_in),
);

initial begin

	mem[0][0] = 1'b1;
	mem[1] = 0;
	mem[2] = 0;
	mem[3] = 0;
	mem[4] = 0;
	mem[5] = 0;
	mem[6] = 0;
	mem[7] = 0;
	mem[8] = 0;
	mem[9] = 0;
	mem[10] = 0;
	mem[11] = 0;
	mem[12] = 0;
	mem[13] = 0;
	mem[14] = 0;
end

wire clk_200m;
wire lock;

    SB_PLL40_CORE #(
        .FEEDBACK_PATH("SIMPLE"),
        .PLLOUT_SELECT("GENCLK"),
        .DIVR(4'b0000),
        .DIVF(7'b0000111),
        .DIVQ(3'b010),
        .FILTER_RANGE(3'b101)
    ) uut (
        .LOCK(lock),
        .RESETB(1'b1),
        .BYPASS(1'b0),
        .REFERENCECLK(clk),
        .PLLOUTCORE(clk_200m)
    );

/*
pwm pwm1 (
    .rst(mem[0][0]),
    .en(mem[0][1]),
    .period(period),
    .duty_cycle(duty_cycle),
    .polarity(mem[0][2]),
    .clk(clk_200m),
    .out(pmod1[0]),
);
*/
endmodule
