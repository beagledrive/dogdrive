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
reg [24:0] counter=0;

wire[ADDR_WIDTH-1:0]  addr;
reg [DATA_WIDTH-1:0]  data_out;
wire [DATA_WIDTH-1:0]  data_in;

// set period to PWM period
//parameter PERIOD = 1/10000;
parameter PERIOD_FPGA = 5;
parameter PERIOD_CLK = 50000; //PERIOD*1000000000/PERIOD_FPGA;  // 10kHz period with 200MHz clock is 20000 cycles
parameter IRQ_PERIOD = 500;  
parameter IRQ_TRIGGER = 0; // IRQ start
parameter DVALID_TRIGGER = 50; // Time before end period when valid data required


wire [15:0] sw_on_mem;
wire [15:0] sw_off_mem;
wire data_valid_mem; // dsp sets to 1 on data transfer, fpga resets at start of period
wire pwm_en; 
wire pwm_re; // Reset
wire pwm_pol; // polarity

assign sw_on_mem[15:0] = mem[2];
//assign sw_on_mem[31:16] = mem[3];
assign sw_off_mem[15:0] = mem[3];
//assign sw_off_mem[31:16] = mem[1];
assign data_valid_mem = mem[0][3];
assign pwm_en = mem[0][1];
assign pwm_re = mem[0][0];
assign pwm_pol = mem[0][2];

reg [15:0] sw_on_next;
reg [15:0] sw_off_next;
reg [15:0] sw_on_curr;
reg [15:0] sw_off_curr;
reg data_valid;

reg irq_signal;
reg pwm_a_out;

assign irq_in = irq_signal;
assign pmod1[0] = pwm_a_out;

//
reg data_valid_prev;
reg data_valid_curr;
reg pwm_fpga_en;
reg pwm_fpga_en_latch;

// Main Routine
always @ (posedge clk)
begin
	// Reset the PWM enable
	if (pwm_en && pwm_re) begin
		pwm_fpga_en_latch <= 1'b1;
	end
	if (pwm_en && !pwm_re && pwm_fpga_en_latch) begin
		pwm_fpga_en <= 1'b1;
		data_valid_curr <= data_valid_mem;
		data_valid_prev <= data_valid_mem+1'b1;
		pwm_fpga_en_latch <= 0;
	end
	// IRQ occurs at start period - could change to before start
	if (counter == IRQ_TRIGGER) begin
		irq_signal <= 1;
	end else if (counter == (IRQ_TRIGGER+IRQ_PERIOD)) begin
		irq_signal <= 0;
	end
		
	// Complete PWM switching
	if (counter >= sw_off_curr) begin
		pwm_a_out <= 1'b0;
	end else if (counter >= sw_on_curr) begin
		pwm_a_out <= 1'b1;
	end else begin
		pwm_a_out <= 1'b0;
	end
	
	// Complete data check and protection before end period 
	if (counter == (PERIOD_CLK - DVALID_TRIGGER)) begin	
		// HOLD - protection functions
		// Currently just set the registers to be the memory location
		// value in sw on mem is a clock counter (ns)
		data_valid_curr <=  data_valid_mem;
		sw_on_next <= sw_on_mem;
		sw_off_next <= sw_off_mem;
	end

	// Remove if breaks
	if ((data_valid_curr != data_valid_prev)) begin
		data_valid <= 1'b1;
	end else begin
		data_valid <= 1'b0;
	end

	// At end of period set the next switches to the current 
	if ((counter == PERIOD_CLK)) begin 
		sw_on_curr <= sw_on_next;
		sw_off_curr <= sw_off_next;
		data_valid_prev <= data_valid_curr;	
	end
/*
	if ((counter == PERIOD_CLK) && !data_valid) begin
		sw_on_curr <= 0;
		sw_off_curr <= 0;
		pwm_fpga_en <= 0;
	end	
*/	// Reset or increment counter
	if (counter == PERIOD_CLK) begin
		counter <= 0;
	end else begin
		counter <= counter + 1'b1;
	end
end


// terminology is from DSP / GPMC perspective
// write enable so set memory address to GPMC pin data
always @ (posedge clk)
begin
    if (!cs && !we && oe) begin
        mem[addr] <= data_out;
	mem[4] <= sw_on_curr;
	mem[5] <= sw_off_curr;
	mem[6][0] <= data_valid;
	mem[6][1] <= pwm_fpga_en;
	mem[6][2] <= pwm_fpga_en_latch;
    end 
end

// output enable so set the data in to the memory address
always @ (posedge clk)
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
//	mem[3] = 0;
//	mem[4] = 0;
//	mem[5] = 0;

	pwm_fpga_en = 0;
	sw_on_curr = 0;
	sw_off_curr = 0;
	counter = 0;
	pwm_a_out = 0;
	pwm_fpga_en_latch = 0;
//	data_valid = 0;
//	irq_signal = 0;

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
