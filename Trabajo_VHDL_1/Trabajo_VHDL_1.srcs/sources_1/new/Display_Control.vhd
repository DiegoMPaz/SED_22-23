----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 29.12.2022 10:45:05
-- Design Name: 
-- Module Name: Display_Control - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;


entity Display_Control is
    PORT(num : IN std_logic_vector(3 DOWNTO 0);
         clk: IN std_logic;
         Venerr:in std_logic_vector(1 downto 0);
         digsel : OUT std_logic_vector(7 DOWNTO 0);
         segmentos : OUT std_logic_vector(7 DOWNTO 0)
        );
end Display_Control;

architecture Behavioral of Display_Control is
    signal numero : unsigned(num'range);
    signal digitos: natural range 0 to 7 :=0;
    signal counter: natural range 0 to 18 :=0;
    signal clk_counter: natural range 0 to 20000 :=0;
begin
    process(clk)
    begin
        if rising_edge(clk) then
            clk_counter<=clk_counter + 1;
            if clk_counter>=20000 then
                clk_counter<=0;
                digitos<=digitos +1;
            end if;
            if digitos > 7 then
                digitos<=0;
            end if;
        end if;

    end process;

    process(digitos)
    begin
        numero <= unsigned(num);
        if numero<=13 then
            if numero=0 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10000001";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                     when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=1 then --0.1€
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="11001111";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                     when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=2 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10010010";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=3 then --0.3€
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10000110";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=4 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="11001100";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=5 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10100100";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=6 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10100000";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=7 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10001111";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=8 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10000000";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=9 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10000100";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="00000001";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=10 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="10000001";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="01001111";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=11 then
                case digitos is
                    when 0=>
                        segmentos<="10000001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="11001111";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="01001111";
                        digsel <="11111011";
                    when 4=>
                        segmentos<="10000001";
                        digsel <="11101111";
                     when 5=>
                        segmentos<="10010010";
                        digsel <="11011111";
                    when 6=>
                        segmentos<="01001111";
                        digsel <="10111111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;

            if numero=12 then
                --if Venerr="10" then
                    case digitos is
                        when 0=>
                            digsel <="11111110";
                            segmentos<="10000001";
                        when 1=>
                            digsel <="11111101";
                            segmentos<="11110001";
                        when 2=>
                            digsel <="11111011";
                            segmentos<="10110000";
                        when 3=>
                            digsel <="11110111";
                            segmentos<="10100001";
                        when 4=>
                            digsel <="11101111";
                            segmentos<="10000001";
                        when 5=>
                            digsel <="11011111";
                            segmentos<="10110001";
                        when others=>
                            digsel<="11111111";
                            segmentos<="11111111";
                    end case;
            end if;

            if numero=13 then
                --if Venerr="10" then
                case digitos is
                    when 0=>
                        segmentos<="11110001";
                        digsel <="11111110";
                    when 1=>
                        segmentos<="11001111";
                        digsel <="11111101";
                    when 2=>
                        segmentos<="10001000";
                        digsel <="11111011";
                    when 3=>
                        segmentos<="10111000";
                        digsel <="11110111";
                    when others=>
                        segmentos<="11111111";
                        digsel<="11111111";
                end case;
            end if;
        end if;


    end process;

end Behavioral;