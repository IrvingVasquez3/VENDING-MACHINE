
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity mostrar_dispensar_leds is
Port ( 
    clk : in STD_LOGIC;
    reset : in STD_LOGIC;
    empezarCuenta : in STD_LOGIC;
    Confirmar_compra_IN : in std_logic; 
    LEDS : out STD_LOGIC_VECTOR (15 downto 0)
);
end mostrar_dispensar_leds;

architecture Behavioral of mostrar_dispensar_leds is

signal s_state: integer := 0;
signal s_clk_en: std_logic := '0';
signal s_count: integer := 0;
signal numeroCuenta: integer := 0;


signal signal_empezarCuenta : std_logic;
signal signal_terminarCuenta : std_logic := '0';

begin

s_clk_en <= clk and empezarCuenta;

process (s_clk_en, reset)
begin
    if (reset = '1') then -- or empezarCuenta = '0'
        s_state <= 0;
        signal_terminarCuenta <= '0';
        numeroCuenta <= 0;
    elsif ( Confirmar_compra_IN = '0') then
        s_state <= 0;
        numeroCuenta <= 0;
    elsif (s_clk_en'event and s_clk_en='1') and Confirmar_compra_IN = '1' then --rising_edge(s_clk_en)
            if (s_state < 18) then
                s_state <= s_state + 1;
                signal_terminarCuenta <= '0';   
                --numeroCuenta <= 0;             
            elsif (s_state >= 18) and Confirmar_compra_IN = '1' then
                signal_terminarCuenta <= '1';
                numeroCuenta <= numeroCuenta + 1;
            elsif Confirmar_compra_IN = '0' then
                s_state <= 0;
                --signal_terminarCuenta <= '1';
            end if;
            
    end if;
end process;

process (s_state,signal_terminarCuenta,numeroCuenta)
begin
if (signal_terminarCuenta = '0' and numeroCuenta < 1) then     -- and numeroCuenta >= 1 --signal_terminarCuenta = '0' and 
--if (signal_terminarCuenta = '0' and numeroCuenta >= 1) then     -- and numeroCuenta >= 1
    case s_state is 
    when 17 => LEDS <= "0000000000000000" ;
    when 16 => LEDS <= "0000000000000001" ;
    when 15 => LEDS <= "0000000000000011" ;
    when 14 => LEDS <= "0000000000000111" ;
    when 13 => LEDS <= "0000000000001111" ;
    when 12 => LEDS <= "0000000000011111" ;
    when 11 => LEDS <= "0000000000111111" ;
    when 10 => LEDS <= "0000000001111111" ;
    when 9 => LEDS <= "0000000011111111" ;
    when 8 => LEDS <= "0000000111111111" ;
    when 7 => LEDS <= "0000001111111111" ;
    when 6 => LEDS <= "0000011111111111" ;
    when 5 => LEDS <= "0000111111111111" ;
    when 4 => LEDS <= "0001111111111111" ;
    when 3 => LEDS <= "0011111111111111" ;
    when 2 => LEDS <= "0111111111111111" ;
    when 1 => LEDS <= "1111111111111111" ;
    when 0 => LEDS <= "0000000000000000"; 
    when others => LEDS <= "0000000000000000";
    end case;
else --if (signal_terminarCuenta = '1') then 
    LEDS <= "0000000000000000";
end if;
end process;



end Behavioral;
