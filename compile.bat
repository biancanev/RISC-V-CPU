@echo off
REM List all your Verilog files here
iverilog -o cpu_sim ^
  src\alu.v ^
  src\register_file.v ^
  src\control_unit.v ^
  src\pipeline_register.v ^
  src\forwarding_unit.v ^
  src\hazard_detection_unit.v ^
  src\immediate_gen.v ^
  src\alu_control.v ^
  src\branch_predictor.v ^
  src\pipeline_control.v ^
  src\risc_v_cpu.v ^
  tb\risc_v_pipeline_cpu_tb.v

REM Run the simulation
vvp cpu_sim