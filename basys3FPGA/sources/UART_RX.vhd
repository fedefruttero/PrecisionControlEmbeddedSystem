 library ieee;
 use ieee.std_logic_1164.ALL;
 use ieee.numeric_std.all;

-- This file contains the UART Receiver.  This receiver is able to
-- receive 8 bits of serial data, one start bit, one stop bit,
-- and no parity bit.  When receive is complete o_rx_dv will be
-- driven high for one clock cycle.

-- Set Generic g_CLKS_PER_BIT as follows:
-- g_CLKS_PER_BIT = (Frequency of i_Clk)/(Frequency of UART)
-- For us: 100 MHz Clock, 9600 baud UART
-- (100000000)/(9600) = 10416


entity UART_RX is
  generic (
    g_CLKS_PER_BIT : integer := 10416     -- Number of clock cycles to transfer one bit 
    );
  port (
    i_Clk       : in  std_logic;          -- Clock input
    i_RX_Serial : in  std_logic;          -- Serial data input
    o_RX_DV     : out std_logic;         -- Data valid signal
    o_RX_Byte   : out std_logic_vector(7 downto 0);  -- Received data byte
    r_RX_7seg   : out std_logic;
    led0_idle   : out std_logic;
    led1_startbit : out std_logic;
    led2_databits : out std_logic;
    led3_stopbit : out std_logic  
    );
end UART_RX;

architecture BEHAVIORAL of UART_RX is

  type RX_stateMachine is (s_Idle, s_RX_Start_Bit, s_RX_Data_Bits,
                     s_RX_Stop_Bit, s_Cleanup);
  signal RX_currentSate : RX_stateMachine := s_Idle;

  signal r_RX_Data_R : std_logic := '0';  -- Registered incoming data
  signal r_RX_Data   : std_logic := '0';  -- Data for UART RX Clock Domain

  signal r_Clk_Count : integer range 0 to g_CLKS_PER_BIT-1 := 0;  -- Counter for clock cycles
  signal r_Bit_Index : integer range 0 to 7 := 0;  -- Bit index (8 bits total)
  signal DATABYTE_TO_7SEG   : std_logic_vector(7 downto 0);  -- Received data byte
  signal r_RX_DV     : std_logic := '0';  -- Data valid signal
  signal led0 : std_logic := '0';
  signal led1 : std_logic := '0';
  signal led2 : std_logic := '0';
  signal led3 : std_logic := '0';

begin

  -- Purpose: Double-register the incoming data.
  -- This allows it to be used in the UART RX Clock Domain.
  -- (It removes problems caused by metastability)
  p_SAMPLE : process (i_Clk)
  begin
    if rising_edge(i_Clk) then
      r_RX_Data_R <= i_RX_Serial;
      r_RX_Data   <= r_RX_Data_R;
    end if;
  end process p_SAMPLE;

  -- Purpose: Control RX state machine
  p_UART_RX : process (i_Clk)
  begin
    if rising_edge(i_Clk) then
      case RX_currentSate is
        when s_Idle =>
          r_RX_DV     <= '0';  -- Clear data valid signal
          r_Clk_Count <= 0;     -- Reset clock cycle count
          r_Bit_Index <= 0;     -- Reset bit index

          if r_RX_Data = '0' then       -- Start bit detected
            RX_currentSate <= s_RX_Start_Bit;
          else
            RX_currentSate <= s_Idle;
            led0 <= '1';
            led1 <= '0';
            led2 <= '0';
            led3 <= '0';

          end if;

        -- Check middle of start bit to make sure it's still low
        when s_RX_Start_Bit =>
          r_RX_7seg   <= '1';
          if r_Clk_Count = (g_CLKS_PER_BIT-1)/2 then
            if r_RX_Data = '0' then
              r_Clk_Count <= 0;  -- Reset counter since we found the middle
              RX_currentSate   <= s_RX_Data_Bits;
            led1 <= '1';
            else
              RX_currentSate   <= s_Idle;
            end if;
          else
            r_Clk_Count <= r_Clk_Count + 1;
            RX_currentSate   <= s_RX_Start_Bit;
          end if;

        -- Wait g_CLKS_PER_BIT-1 clock cycles to sample serial data
        when s_RX_Data_Bits =>
          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            RX_currentSate   <= s_RX_Data_Bits;
            led2 <= '1';
          else
            r_Clk_Count            <= 0;
            DATABYTE_TO_7SEG(r_Bit_Index) <= r_RX_Data;
             
            -- Check if we have sent out all bits
            if r_Bit_Index < 7 then
              r_Bit_Index <= r_Bit_Index + 1;
              RX_currentSate   <= s_RX_Data_Bits;
            else
              r_Bit_Index <= 0;
              RX_currentSate   <= s_RX_Stop_Bit;
            end if;
          end if;
           
        -- Receive Stop bit. Stop bit = 1
        when s_RX_Stop_Bit =>
          led3 <= '1';
          -- Wait g_CLKS_PER_BIT-1 clock cycles for Stop bit to finish
          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            RX_currentSate   <= s_RX_Stop_Bit;
          else
            r_RX_DV     <= '1';  -- Set data valid signal
            r_Clk_Count <= 0;     -- Clear clock cycle count
            RX_currentSate   <= s_Cleanup;
          end if;

        -- Stay here for 1 clock cycle
        when s_Cleanup =>
          RX_currentSate <= s_Idle;
          r_RX_DV   <= '0';  -- Clear data valid signal
        when others =>
          RX_currentSate <= s_Idle;

      end case;
    end if;
  end process p_UART_RX;

  o_RX_DV       <=    r_RX_DV;
  o_RX_Byte     <=    DATABYTE_TO_7SEG;
  led0_idle     <=    led0;
  led1_startbit <=    led1;
  led2_databits <=    led2;
  led3_stopbit  <=    led3;
   
end BEHAVIORAL;
