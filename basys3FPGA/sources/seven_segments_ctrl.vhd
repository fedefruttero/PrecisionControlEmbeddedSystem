library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
entity DisplayController is
    Port ( clk_100MHz       : in    STD_LOGIC;
           enableDisplay    : in    STD_LOGIC;
           dataValid        : in    STD_LOGIC;
           temperature      : in    STD_LOGIC_VECTOR(7 downto 0);
           seg              : out   STD_LOGIC_VECTOR(6 downto 0);
           anode            : out   STD_LOGIC_VECTOR(3 downto 0));
end DisplayController;

architecture Behavioral of DisplayController is
    -- Segment patterns for digits 0-9
    type Seg7_Array is array (0 to 9) of STD_LOGIC_VECTOR(6 downto 0);
    constant Seg7_Patterns: Seg7_Array := (
        "0000001", -- 0
        "1001111", -- 1
        "0010010", -- 2
        "0000110", -- 3
        "1001100", -- 4
        "0100100", -- 5
        "0100000", -- 6
        "0001111", -- 7
        "0000000", -- 8
        "0000100"  -- 9
    );

    signal count : integer range 0 to 2 := 0; -- Count to select between the two displays
    signal digit_timer : integer range 0 to 99999;
    signal tens_digit : natural := 0; -- Tens digit
    signal ones_digit : natural := 0; -- Ones digit
    signal display_num : STD_LOGIC_VECTOR(6 downto 0) := (others => '1'); -- Default: All segments off
begin

    process (clk_100MHz, dataValid)
    begin
        if rising_edge(clk_100MHz) then
            if dataValid = '1' then
                -- Extract tens and ones digits (BCD representation)
                tens_digit <= to_integer(unsigned(temperature(7 downto 4)));
                ones_digit <= to_integer(unsigned(temperature(3 downto 0)));
            end if;
        end if;
    end process;

    process (clk_100MHz, enableDisplay)
    begin
        if enableDisplay = '0' then
            display_num <= (others => '1'); -- All segments off
        elsif rising_edge(clk_100MHz) then
            -- Alternate between the two displays
            if count = 0 then
                display_num <= Seg7_Patterns(tens_digit);
                anode <= "1011"; -- Enable the first display
            elsif count = 1 then
                display_num <= Seg7_Patterns(ones_digit);
                anode <= "1110"; -- Enable the second display
            else
                display_num <= "0011100";  -- °
                anode <= "1101"; -- Enable the third display showing that it's a decimal number
            end if;
        end if;
    end process;

    process (clk_100MHz, enableDisplay)
    begin
        if enableDisplay = '0' then
            count <= 0;
            digit_timer <= 0;
        elsif rising_edge(clk_100MHz) then
            if(digit_timer = 99999) then -- The period of 100MHz clock is 10ns (1/100,000,000 seconds)
                digit_timer <= 0;
                if (count = 2) then
                    count <= 0;
                else
                    count <= count + 1;
                end if;
            else
                digit_timer <= digit_timer + 1;
            end if;
        end if;
    end process;

    -- Assign segment patterns to the 7-segment displays
    seg <= display_num;

end Behavioral;
