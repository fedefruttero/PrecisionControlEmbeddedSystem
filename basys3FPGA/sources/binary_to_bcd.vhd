
--This VHDL code implements a binary to binary-coded decimal (BCD) conversion. 
--The process unfolds in a finite state machine with three distinct states: start, shift, and done. 
--During the start state, the binary input is initialized, and the BCD value is set to zero. The subsequent
--shift state systematically shifts the binary input leftward, constructing the BCD value. Following the
--shift, the done state ensures BCD correctness by adjusting each digit if it surpasses 4. The corrected
--BCD output is then available through the bcd_out port. The state machine seamlessly transitions through
--these states, perpetuating the conversion process on each clock cycle. This well-orchestrated mechanism
--allows for efficient and accurate binary to BCD conversion in digital systems.

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
 
entity binary_bcd is
    generic(N: positive := 8);
    port(
        clk, reset: in std_logic;
        binary_in: in std_logic_vector(N-1 downto 0);
        bcd_out: out std_logic_vector(N-1 downto 0)
    );
end binary_bcd;
 
architecture BEHAVIORAL of binary_bcd is
    type states is (start, shift, done);
    signal state, state_next: states;
 
    signal binary, binary_next: std_logic_vector(N-1 downto 0);
    signal bcds, bcds_reg, bcds_next: std_logic_vector(7 downto 0);
    -- output register keep output constant during conversion
    signal bcds_out_reg, bcds_out_reg_next: std_logic_vector( downto 0);
    -- need to keep track of shifts
    signal shift_counter, shift_counter_next: natural range 0 to N;
begin
    process(clk, reset)
    begin
        if reset = '1' then
            -- Initialize values on reset
            binary <= (others => '0');
            bcds <= (others => '0');
            state <= start;
            bcds_out_reg <= (others => '0');
            shift_counter <= 0;
        elsif falling_edge(clk) then
            -- Update values on falling edge of the clock
            binary <= binary_next;
            bcds <= bcds_next;
            state <= state_next;
            bcds_out_reg <= bcds_out_reg_next;
            shift_counter <= shift_counter_next;
        end if;
    end process;

    convert:
    process(state, binary, binary_in, bcds, bcds_reg, shift_counter)
    begin
        -- Default assignments for next state
        state_next <= state;
        bcds_next <= bcds;
        binary_next <= binary;
        shift_counter_next <= shift_counter;

        case state is
            when start =>
                -- Start state initializes variables for conversion
                state_next <= shift;
                binary_next <= binary_in;
                bcds_next <= (others => '0');
                shift_counter_next <= 0;
            when shift =>
                -- Shift binary digits into BCD
                if shift_counter = N then
                    state_next <= done;
                else
                    binary_next <= binary(N-2 downto 0) & 'L';
                    bcds_next <= bcds_reg(6 downto 0) & binary(N-1);
                    shift_counter_next <= shift_counter + 1;
                end if;
            when done =>
                -- Reset state after conversion is done
                state_next <= start;
        end case;
    end process;

    -- BCD correction based on 3, add 3 to BCD digit if it's greater than 4
    bcds_reg(7 downto 4) <= bcds(7 downto 4) + 3 when bcds(7 downto 4) > 4 else
                            bcds(7 downto 4);
    bcds_reg(3 downto 0) <= bcds(3 downto 0) + 3 when bcds(3 downto 0) > 4 else
                            bcds(3 downto 0);

    -- Update output register only when conversion is done
    bcds_out_reg_next <= bcds when state = done else
                         bcds_out_reg;

    -- Output the final BCD value
    bcd_out <= bcds_out_reg(7 downto 0);
 
end BEHAVIORAL;
