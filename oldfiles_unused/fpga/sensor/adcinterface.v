
module adcinterface(
  input wire clk,
  output reg SDA,
  output wire SCL
);
  
  reg [7:0] state ;
  reg begin_process = 1;
  reg begin_clock = 1;
  reg scl_enable = 0;
  
  reg [6:0] address;
  reg [7:0] count_addr; 
  reg [7:0] command;
  reg [7:0] count_command;
  reg [7:0] sensorData;
  reg [7:0] sensorData_length;
  
  assign SCL = (scl_enable == 0) ? 1 : ~clk;
  
  localparam idle = 0;
  localparam start = 1;
  localparam send_address = 2;
  localparam send_write = 3;
  localparam ack1 = 4;
  localparam write_command = 5;
  localparam ack2 = 6;
  localparam sr_start= 7;
  localparam send_address_again= 8;
  localparam send_read= 9;
  
   localparam readSensor1a= 10;
   localparam ack3a= 11;
  localparam readSensor2a= 14;
   localparam ack4a= 15;
  localparam readSensor3a= 18;
   localparam ack5a= 19;
  localparam readSensor4a= 22;
   localparam ack6a= 23;
  
    
   localparam readSensor1b= 12;
   localparam ack3b= 13;
  localparam readSensor2b= 16;
   localparam ack4b= 17;
  localparam readSensor3b= 20;
   localparam ack5b= 21;
  localparam readSensor4b= 25;
   localparam ack6b= 25;
  
  
  localparam stop = 26;
  
  reg ackstate1 = 0;
  reg ackstate2 = 0;
  reg ackstate3a = 0;
  reg ackstate4a = 0;
  reg ackstate5a = 0;
  reg ackstate6a = 0;
  reg ackstate3b = 0;
  reg ackstate4b = 0;
  reg ackstate5b = 0;
  reg ackstate6b = 0;
  reg SDA_in_ack = 1;
    
  always @(negedge clk) begin
    if (begin_clock ==1) begin
      scl_enable = 0;
      begin_clock <= 0;
    end  
    else begin
      
      if ( (state == idle) || (state == start) || ( state == stop) || (state == ack1)|| (state == ack2)|| (state == ack3a)|| (state == ack4a)|| (state == ack5a)|| (state == ack6a )|| (state == ack3b)|| (state == ack4b)|| (state == ack5b)|| (state == ack6b) ) begin
         scl_enable = 0;
      end//if
      else begin
         scl_enable = 1;
      end //else
        
    end
    
    
  end// clock always
  
  
  
  always @(posedge clk) begin
    
    if (begin_process ==1) begin
      state <=idle;
      address = 7'h20;
      sensorData = 8'hCC;
      count_addr= 8'd0;
      sensorData_length = 8'd0;
      
      command = 8'hF0;
      count_command= 8'd0;
      
      begin_process = 0;
    end
     
    case (state)
        
       idle : begin //0
         SDA <= 1;    
         state <= start;
       end //idle
     
       start : begin//1
         SDA <= 0;        
         state <= send_address;
         count_addr <= 6;
       end // start
      
      send_address : begin//2
        SDA <= address[count_addr];
        if ( count_addr == 0)  state <= send_write; 
        else  count_addr <= count_addr -1 ; 
      end // send address bit
      
      send_write : begin //3
         SDA <= 0; //write command
         state <= ack1 ;
       end // start
      
      ack1 : begin //4
         SDA <= 0; 
         SDA_in_ack = SDA;
        if(SDA == 0) ackstate1 <= 1; scl_enable = 1;
         state <= write_command ;
        count_command <= 7;
       end // start
        
      write_command : begin //5
        SDA <= command[count_command];
        if (count_command == 0)  state <= ack2; 
        else  count_command <= count_command -1 ; 
      end // write_command
      
       ack2 : begin //6
         SDA <= 0;
         SDA_in_ack = SDA;
         if(SDA_in_ack == 0) ackstate2 <= 1; scl_enable = 1;
        state <= sr_start ;
       end // ack2
      
      sr_start : begin//7
         SDA <= 0;        
         state <= send_address_again;
         count_addr <= 6;
       end // sr_start
      
      send_address_again : begin//8
        SDA <= address[count_addr];
        if ( count_addr == 0)  state <= send_read; 
        else  count_addr <= count_addr -1 ; 
      end // send address bit
      
      send_read : begin //9
         SDA <= 1; //read command
         state <= readSensor1a ;
        sensorData_length <= 7;
       end // start
      
         
      readSensor1a : begin //10
           
        SDA <= sensorData[sensorData_length];
        if ( sensorData_length == 0)  state <= ack3a; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
      
       ack3a : begin //11
         SDA <= 0;
         SDA_in_ack = SDA;
         if(SDA_in_ack == 0) ackstate3a <= 1; scl_enable = 1;
        state <= readSensor1b ;
        sensorData_length <= 7;
       end // ack3
      
      readSensor1b : begin //10
           
        SDA <= sensorData[sensorData_length];
        if ( sensorData_length == 0)  state <= ack3b; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
      
       ack3b : begin //11
         SDA <= 0;
         SDA_in_ack = SDA;
         if(SDA_in_ack == 0) ackstate3b <= 1; scl_enable = 1;
        state <= readSensor2a ;
        sensorData_length <= 7;
       end // ack3
      
      
      
      
      readSensor2a : begin //12
           
        SDA <= sensorData[sensorData_length];
        if ( sensorData_length == 0)  state <= ack4a; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
     
         ack4a : begin //13
         SDA <= 0;
         SDA_in_ack = SDA;
           if(SDA_in_ack == 0) ackstate4a <= 1; scl_enable = 1;
        state <= readSensor2b ;
        sensorData_length <= 7;
       end // ack3
      
       readSensor2b : begin //14
           
        SDA <= sensorData[sensorData_length];
         if ( sensorData_length == 0)  state <= ack4b; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
     
         ack4b : begin //15
         SDA <= 0;
         SDA_in_ack = SDA;
           if(SDA_in_ack == 0) ackstate4b <= 1; scl_enable = 1;
        state <= readSensor3a ;
        sensorData_length <= 7;
       end // ack3
      
      
      readSensor3a : begin //16
           
        SDA <= sensorData[sensorData_length];
        if ( sensorData_length == 0)  state <= ack5a; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
     
         ack5a : begin //17
         SDA <= 0;
         SDA_in_ack = SDA;
           if(SDA_in_ack == 0) ackstate5a <= 1; scl_enable = 1;
        state <= readSensor3b ;
        sensorData_length <= 7;
       end // ack3
      
      readSensor3b : begin //18
           
        SDA <= sensorData[sensorData_length];
        if ( sensorData_length == 0)  state <= ack5b; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
     
         ack5b : begin //19
         SDA <= 0;
         SDA_in_ack = SDA;
           if(SDA_in_ack == 0) ackstate5b <= 1; scl_enable = 1;
        state <= readSensor4a ;
        sensorData_length <= 7;
       end // ack3
      
      
      
      readSensor4a : begin //20
           
        SDA <= sensorData[sensorData_length];
        if ( sensorData_length == 0)  state <= ack6a; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
     
         ack6a : begin //21
         SDA <= 0;
         SDA_in_ack = SDA;
           if(SDA_in_ack == 0) ackstate6a <= 1; scl_enable = 1;
        state <= stop ;
         end
           
             readSensor4b : begin //22
           
        SDA <= sensorData[sensorData_length];
               if ( sensorData_length == 0)  state <= ack6b; 
        else  sensorData_length <= sensorData_length -1 ; 
       
       end // start
      
     
         ack6b : begin //23
         SDA <= 0;
         SDA_in_ack = SDA;
           if(SDA_in_ack == 0) ackstate6b <= 1; scl_enable = 1;
        state <= stop ;
           
        
       end // ack3
      
       stop : begin //24
        SDA <= 1;
        state <= idle ;
       end // ack2
      
    endcase //case
      
end //always

  
endmodule
  
  