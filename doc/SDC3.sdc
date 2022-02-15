create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]
create_clock -name cam0_pclk -period 20.000 [get_ports {cam0_pclk}]
create_clock -name cam1_pclk -period 20.000 [get_ports {cam1_pclk}]