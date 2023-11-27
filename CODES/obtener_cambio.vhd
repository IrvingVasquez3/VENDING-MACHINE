
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity obtener_cambio is
Port ( 
    clk_100mhz : in std_logic;
    Dispensar_out : in std_logic;
    Confirmar : in std_logic;
    dineroAcumulado : in integer;
    precioProducto : in integer;
    valor_del_cambio : out integer
);
end obtener_cambio;

architecture Behavioral of obtener_cambio is

begin
    process(clk_100mhz)
    begin
    
    if (Confirmar = '1') then
	       valor_del_cambio <= dineroAcumulado - precioProducto;
       else
           -- valor_del_cambio <= 99;
       end if;
    
	if (clk_100mhz'event and clk_100mhz='1') then
	   --valor_del_cambio <= dineroAcumulado - precioProducto;
	   
        
	end if;
    end process;
    

end Behavioral;
