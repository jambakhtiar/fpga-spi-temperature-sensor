library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity spi_adc_tb is end entity;

architecture bench of spi_adc_tb is
	
	-- number of blank bits provided by the ADC
	constant NBLANKBITS : positive := 1;
	
	-- SCK = CLK_250_MHZ / (POSTSCALER*2) = 62.5 MHz
	constant SCK_POSTSCALER : std_logic_vector := "0000000000000010";
	
	-- main clock period
	constant CLK_PERIOD : time := 4.0 ns; -- 250 MHz
	
	-- simulated data sample produced by the ADC
	signal rawdata : unsigned(13 downto 0) := (others=>'0');
	
	-- clock signals
	signal clk_250, sampling_pulse : std_logic := '0';
	
	-- SPI signals
	signal SPI_DIN, SPI_nCS, SPI_CLK : std_logic := '0';
	
	begin
		
		primary_clock: clk_250 <= not clk_250 after CLK_PERIOD / 2;
		
		--------------------------------------------------------------------------------
		-- DEVICE UNDER TEST
		--------------------------------------------------------------------------------
		
		DUT: entity work.spi_adc
		port map(
			clk_250 => clk_250,
			sampling_pulse => sampling_pulse,
			postscaler_in => SCK_POSTSCALER,
			spi_sck => SPI_CLK,
			spi_cs_n => SPI_nCS,
			spi_din => SPI_DIN,
			data_out => open);
		
		--------------------------------------------------------------------------------
		-- ANALOG-TO-DIGITAL CONVERTER MODEL
		--------------------------------------------------------------------------------
		
		DATA_SAMPLE: process
		begin
			wait for CLK_PERIOD*100;
			
			rawdata <= to_unsigned(12345,14);
			sampling_pulse <= '1';
			wait for CLK_PERIOD;
			sampling_pulse <= '0';
			
			wait for CLK_PERIOD*100;
			
			rawdata <= to_unsigned(5782,14);
			sampling_pulse <= '1';
			wait for CLK_PERIOD;
			sampling_pulse <= '0';
			
			wait for CLK_PERIOD*100;
			
			rawdata <= to_unsigned(777,14);
			sampling_pulse <= '1';
			wait for CLK_PERIOD;
			sampling_pulse <= '0';
			
		end process DATA_SAMPLE;
		
		SPI_TARGET: process(SPI_nCS,SPI_CLK,SPI_DIN)
		variable counter : integer := 0;
		begin
			if SPI_nCS='1' then
				SPI_DIN <= 'Z';
				counter := 13 + NBLANKBITS;
			elsif SPI_nCS='0' and falling_edge(SPI_CLK) then
				if (counter > 13 or counter < 0) then
					SPI_DIN <= '0';
				else
					SPI_DIN <= std_logic(rawdata(counter));
				end if;
				counter := counter - 1;
			end if;
		end process SPI_TARGET;
		
	end architecture bench;