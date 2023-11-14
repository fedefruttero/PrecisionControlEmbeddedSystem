library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
 
entity UART_top is
    port(
        clk_100MHz      : in    std_logic;
        i_RX_Serial     : in    std_logic;
        enableDisplay   : in    std_logic;
        seg             : out   STD_LOGIC_VECTOR(6 downto 0);
        anode           : out   STD_LOGIC_VECTOR(3 downto 0);
        led0_idle       : out std_logic;
        led1_startbit   : out std_logic;
        led2_databits   : out std_logic;
        led3_stopbit    : out std_logic
    );
end UART_top;
 
architecture top of UART_top is
 

 
  component UART_RX is
  generic (
    g_CLKS_PER_BIT : integer := 10416     -- Number of clock cycles per bit (should be set correctly)
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
end component;
  
  component binary_bcd is
    generic(N: positive := 8);
    port(
        clk, reset: in std_logic;
        binary_in: in std_logic_vector(N-1 downto 0);
        bcd_out: out std_logic_vector(7 downto 0)
    );
  end component;

  component DisplayController is
    Port ( clk_100MHz       : in    STD_LOGIC;
           enableDisplay    : in    STD_LOGIC;
           dataValid        : in    STD_LOGIC;
           temperature      : in    STD_LOGIC_VECTOR(7 downto 0);
           seg              : out   STD_LOGIC_VECTOR(6 downto 0);
           anode            : out   STD_LOGIC_VECTOR(3 downto 0)
        );
end component;
   
  -- top Bench uses a 100 MHz Clock
  -- Want to interface to 9600 baud UART
  -- 100000000 / 9600 = 10416 Clocks Per Bit.
  constant c_CLKS_PER_BIT : integer := 10416;
  signal DATA_BYTE : std_logic_vector(7 downto 0);
  signal o_RX_DV          :     std_logic;
  signal o_RX_Byte        :     std_logic_vector(7 downto 0);
  signal TX_done          :     std_logic;
  signal r_RX_7segs       :     std_logic := '0';
  signal bcd_out_s        :     std_logic_vector(7 downto 0);
  signal led0 : std_logic := '0';
  signal led1 : std_logic := '0';
  signal led2 : std_logic := '0';
  signal led3 : std_logic := '0';
begin
 
--Instantiate 7segmentsController
  SEVEN_SEG_CTRL : DisplayController
    port map(
      clk_100MHz,
      enableDisplay,
      r_RX_7segs,
      bcd_out_s,
      seg,
      anode
    );

  BINARY_TO_BCD: binary_bcd
  port map(
    clk => clk_100MHz,
    reset => '0',
    binary_in => o_RX_Byte,
    bcd_out => bcd_out_s
  );
  -- Instantiate UART Receiver
  RECEIVER: UART_RX
    generic map (
      g_CLKS_PER_BIT => 10416  -- Modify as needed for your UART configuration
    )
    port map (
      i_Clk => clk_100MHz,
      i_RX_Serial => i_RX_Serial,
      o_RX_DV => o_RX_DV,
      o_RX_Byte => o_RX_Byte,
      r_RX_7seg => r_RX_7segs,
      led0_idle => led0,
      led1_startbit => led1,
      led2_databits => led2,
      led3_stopbit => led3
    );
  led0_idle <= led0;
  led1_startbit <= led1;
  led2_databits <= led2;
  led3_stopbit <= led3;
   
end top;