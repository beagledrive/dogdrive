

`timescale 1us/100ps
module test;

  reg clk ;
  reg SDA;
  reg SCL;
  
  adcinterface adcinterface(.*);
 
  initial begin
    clk = 0;
  
    
    forever #1 clk = !clk; //1Mhz - 
  end

  initial begin
  
    repeat (200) @(posedge clk);
    $finish;
  end

  initial begin
    $dumpfile("dump.vcd"); $dumpvars;
  end
endmodule
 

 
