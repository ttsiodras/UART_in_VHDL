-- Based on ZestSC1 Example skeleton code, used to demonstrate register interfaces.

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity Example2 is
    port (
        USB_StreamCLK : in std_logic;
        USB_StreamFIFOADDR : out std_logic_vector(1 downto 0);
        USB_StreamPKTEND_n : out std_logic;
        USB_StreamFlags_n : in std_logic_vector(2 downto 0);
        USB_StreamSLOE_n : out std_logic;
        USB_StreamSLRD_n : out std_logic;
        USB_StreamSLWR_n : out std_logic;
        USB_StreamFX2Rdy : in std_logic;
        USB_StreamData : inout std_logic_vector(15 downto 0);

        USB_RegCLK : in std_logic;
        USB_RegAddr : in std_logic_vector(15 downto 0);
        USB_RegData : inout std_logic_vector(7 downto 0);
        USB_RegOE_n : in std_logic;
        USB_RegRD_n : in std_logic;
        USB_RegWR_n : in std_logic;
        USB_RegCS_n : in std_logic;

        USB_Interrupt : out std_logic;

        User_Signals : inout std_logic_vector(7 downto 0);

        S_CLK : out std_logic;
        S_A : out std_logic_vector(22 downto 0);
        S_DA : inout std_logic_vector(8 downto 0);
        S_DB : inout std_logic_vector(8 downto 0);
        S_ADV_LD_N : out std_logic;
        S_BWA_N : out std_logic;
        S_BWB_N : out std_logic;
        S_OE_N : out std_logic;
        S_WE_N : out std_logic;

        IO_CLK_N : inout std_logic;
        IO_CLK_P : inout std_logic;
        IO : inout std_logic_vector(46 downto 0)
    );
end Example2;

architecture arch of Example2 is

    -- Declare interfaces component
    component ZestSC1_Interfaces
        port (
            -- FPGA pin connections
            USB_StreamCLK : in std_logic;
            USB_StreamFIFOADDR : out std_logic_vector(1 downto 0);
            USB_StreamPKTEND_n : out std_logic;
            USB_StreamFlags_n : in std_logic_vector(2 downto 0);
            USB_StreamSLOE_n : out std_logic;
            USB_StreamSLRD_n : out std_logic;
            USB_StreamSLWR_n : out std_logic;
            USB_StreamFX2Rdy : in std_logic;
            USB_StreamData : inout std_logic_vector(15 downto 0);

            USB_RegCLK : in std_logic;
            USB_RegAddr : in std_logic_vector(15 downto 0);
            USB_RegData : inout std_logic_vector(7 downto 0);
            USB_RegOE_n : in std_logic;
            USB_RegRD_n : in std_logic;
            USB_RegWR_n : in std_logic;
            USB_RegCS_n : in std_logic;

            USB_Interrupt : out std_logic;

            S_CLK: out std_logic;
            S_A: out std_logic_vector(22 downto 0);
            S_ADV_LD_N: out std_logic;
            S_BWA_N: out std_logic;
            S_BWB_N: out std_logic;
            S_DA: inout std_logic_vector(8 downto 0);
            S_DB: inout std_logic_vector(8 downto 0);
            S_OE_N: out std_logic;
            S_WE_N: out std_logic;

            -- User connections
            -- Streaming interface
            User_CLK : out std_logic;
            User_RST : out std_logic;

            User_StreamBusGrantLength : in std_logic_vector(11 downto 0);

            User_StreamDataIn : out std_logic_vector(15 downto 0);
            User_StreamDataInWE : out std_logic;
            User_StreamDataInBusy : in std_logic;

            User_StreamDataOut : in std_logic_vector(15 downto 0);
            User_StreamDataOutWE : in std_logic;
            User_StreamDataOutBusy : out std_logic;

            -- Register interface
            User_RegAddr : out std_logic_vector(15 downto 0);
            User_RegDataIn : out std_logic_vector(7 downto 0);
            User_RegDataOut : in std_logic_vector(7 downto 0);
            User_RegWE : out std_logic;
            User_RegRE : out std_logic;

            -- Signals and interrupts
            User_Interrupt : in std_logic;

            -- SRAM interface
            User_SRAM_A: in std_logic_vector(22 downto 0);
            User_SRAM_W: in std_logic;
            User_SRAM_R: in std_logic;
            User_SRAM_DR_VALID: out std_logic;
            User_SRAM_DW: in std_logic_vector(17 downto 0);
            User_SRAM_DR: out std_logic_vector(17 downto 0)
        );
    end component;


    COMPONENT CountingFifo
        PORT (
            clk : IN STD_LOGIC;
            rst : IN STD_LOGIC;
            din : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
            wr_en : IN STD_LOGIC;
            rd_en : IN STD_LOGIC;
            dout : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
            full : OUT STD_LOGIC;
            empty : OUT STD_LOGIC;
            data_count : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
        );
    END COMPONENT;

    ----------------------------
    -- Basic operational signals
    ----------------------------

    signal CLK : std_logic;
    signal RST : std_logic;
    signal LEDs : std_logic_vector(7 downto 0);

    ----------------------
    -- FPGA Bridge signals
    ----------------------

    -- signals connected to the FPGA's USB inputs (coming from PC)
    signal USB_DataIn : std_logic_vector(15 downto 0);
    signal USB_DataInBusy : std_logic;
    signal USB_DataInWE : std_logic;

    -- signals connected to the FPGA's USB outputs (going to PC)
    signal USB_DataOut : std_logic_vector(15 downto 0);
    signal USB_DataOutBusy : std_logic;
    signal USB_DataOutWE : std_logic := '0';

    signal REG_Addr : std_logic_vector(15 downto 0);
    signal REG_DataIn : std_logic_vector(7 downto 0);
    signal REG_DataOut : std_logic_vector(7 downto 0) := X"FF";
    signal REG_WE : std_logic;
    signal REG_RE : std_logic;

    -- The signals connected to the GPIO pins - output...
    signal serial_TX : std_logic := '1';
    -- ...and input
    signal serial_RX : std_logic := '0';
    signal serial_RX_Copy : std_logic := '0';

    -- to lessen noise, use burst protocol for ACTUAL serial_RX
    signal filter : std_logic_vector(127 downto 0) := X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF";
    signal oldBit : std_logic := '1';
    signal cleaned_serial_RX : std_logic := '1';

    ---------------------
    -- UART speed signals
    ---------------------

    constant clock_rate : integer := 48000000;
    constant baud_rate : integer := 115200;
    constant cycles_per_baud : integer := (clock_rate / baud_rate);
    constant cycles_per_baud_d2 : integer := (clock_rate / (2*baud_rate));
    signal baud_count_TX : integer range 0 to cycles_per_baud := 0;
    signal baud_count_RX : integer range 0 to cycles_per_baud := 0;

    ------------------
    -- RX FIFO signals
    ------------------

    -- states of the state machine that drains the UART RX fifo to the USB RX fifo
    type state_RX_type is (
        state_WaitForRxFifo,
        state_WaitForFifoReadToStart,
        state_WaitForFifoToExtract8bits,
        state_WaitForUSBwriteToComplete
    );
    signal state_RX : state_RX_type := state_WaitForRxFifo;

    signal RX_FIFO_ReadEn : std_logic := '0';
    signal RX_FIFO_WriteEn : std_logic := '0';
    signal RX_FIFO_DataOut : std_logic_vector(7 downto 0) := X"00";
    signal RX_FIFO_DataIn : std_logic_vector(7 downto 0) := X"00";
    signal RX_FIFO_Empty : std_logic := '1';
    signal RX_FIFO_Full : std_logic := '0';
    signal RX_FIFO_Count : std_logic_vector(7 downto 0);

    ------------------
    -- TX FIFO signals
    ------------------

    -- states of the state machine that drains the USB TX fifo to the UART TX FIFO
    type state_TX_type is (
        state_WaitForUSBTXdata,
        state_WaitForUSBTXdataToBeCopied,
        state_WaitForTXFifoToWriteData
    );
    signal state_TX : state_TX_type := state_WaitForUSBTXdata;

    signal TX_FIFO_ReadEn : std_logic := '0';
    signal TX_FIFO_WriteEn : std_logic := '0';
    signal TX_FIFO_DataOut : std_logic_vector(7 downto 0) := X"00";
    signal TX_FIFO_DataIn : std_logic_vector(7 downto 0) := X"00";
    signal TX_FIFO_Empty : std_logic := '1';
    signal TX_FIFO_Full : std_logic := '0';
    signal TX_FIFO_Count : std_logic_vector(7 downto 0);

    ------------------
    -- UART TX signals
    ------------------

    -- states of the UART TX
    type state_UART_TX_type is (
        state_Initial_or_StopBit,
        state_StartBit,
        state_SendCharacters
    );
    signal state_UART_TX : state_UART_TX_type := state_Initial_or_StopBit;

    type state_UART_RX_type is (
        state_Initial,
        state_StartBitReceived,
        state_WaitHalfABaud,
        state_WaitForStop
    );
    signal state_UART_RX : state_UART_RX_type := state_Initial;

    signal charBeingSent : std_logic_vector(7 downto 0) := X"00";
    signal bitsSentSoFar : integer range 0 to 7 := 0;

    signal charBeingRead : std_logic_vector(7 downto 0) := X"00";
    signal bitsReceivedSoFar : integer range 0 to 7 := 0;

    -- the flag to execute the UART TX logic every cycles_per_baud
    signal s_UART_TX_step : std_logic := '0';
    -- the flag to execute the UART RX logic every cycles_per_baud
    -- but with a half period shift
    signal s_UART_RX_step : std_logic := '0';

    -- flag for wait state in UART TX draining to GPIO
    signal s_readFromUART_TXFifoOnNextCycle : std_logic := '0';
    signal s_writeToUART_RXFifoOnNextCycle : std_logic := '0';

begin

    -- Tie unused signals
    User_Signals <= "ZZZZZZZZ";
    LEDs(7 downto 1) <= "1111111";
    IO_CLK_N <= 'Z';
    IO_CLK_P <= 'Z';
    -- IO <= (0=>LEDs(0), 1=>LEDs(1), 2=>serial_TX, 3=>serial_RX, 41=>LEDs(2), 42=>LEDs(3), 43=>LEDs(4),
    --        44=>LEDs(5), 45=>LEDs(6), 46=>LEDs(7), others => 'Z');
    IO(0) <= LEDs(0);
    IO(1) <= LEDs(1);
    IO(2) <= serial_TX;
    serial_RX <= IO(3);
    IO(4) <= 'Z';
    -- IO(5) <= 'Z';
    IO(5) <= serial_RX_Copy;
    --cleaned_serial_RX <= serial_RX;
    IO(6) <= 'Z';
    IO(7) <= 'Z';
    IO(8) <= 'Z';
    IO(9) <= 'Z';
    IO(10) <= 'Z';
    IO(11) <= 'Z';
    IO(12) <= 'Z';
    IO(13) <= 'Z';
    IO(14) <= 'Z';
    IO(15) <= 'Z';
    IO(16) <= 'Z';
    IO(17) <= 'Z';
    IO(18) <= 'Z';
    IO(19) <= 'Z';
    IO(20) <= 'Z';
    IO(21) <= 'Z';
    IO(22) <= 'Z';
    IO(23) <= 'Z';
    IO(24) <= 'Z';
    IO(25) <= 'Z';
    IO(26) <= 'Z';
    IO(27) <= 'Z';
    IO(28) <= 'Z';
    IO(29) <= 'Z';
    IO(30) <= 'Z';
    IO(31) <= 'Z';
    IO(32) <= 'Z';
    IO(33) <= 'Z';
    IO(34) <= 'Z';
    IO(35) <= 'Z';
    IO(36) <= 'Z';
    IO(37) <= 'Z';
    IO(38) <= 'Z';
    IO(39) <= 'Z';
    IO(40) <= 'Z';
    IO(41) <= LEDs(2);
    IO(42) <= LEDs(3);
    IO(43) <= LEDs(4);
    IO(44) <= LEDs(5);
    IO(45) <= LEDs(6);
    IO(46) <= LEDs(7);

    serial_RX_Copy <= serial_RX;

    ----------------------------------------------------
    -- processes handling the reading from the registers
    ----------------------------------------------------

    process(REG_Addr, RX_FIFO_Count, RX_FIFO_Empty)
    begin
        if (REG_RE='1') then
            case REG_Addr is
                when X"2000" =>
                    -- The byte currently at the tip of the Q
                    REG_DataOut <= RX_FIFO_DataOut;
                when X"2002" =>
                    -- Return whether the FIFO is empty - but only if
                    -- we aren't currently asking it to read one more byte!
                    REG_DataOut <= (others => RX_FIFO_Empty or RX_FIFO_ReadEn);
                when X"2010" =>
                    -- How many bytes in the Q?
                    REG_DataOut <= RX_FIFO_Count;
                when others =>
                    REG_DataOut <= X"FF";
            end case;
        end if;
    end process;

    process(CLK)
    begin
        if (CLK'event and CLK='1') then
            if (REG_RE='1') then
                case REG_Addr is
                    when X"2000" =>
                        -- When reading the current tip of the Q, if there are
                        -- more bytes in, signal to get the next one
                        if RX_FIFO_Empty = '0' then
                            RX_FIFO_ReadEn <= '1';
                        end if;
                    when others =>
                        RX_FIFO_ReadEn <= '0';
                end case;
            else
                RX_FIFO_ReadEn <= '0';
            end if;
        end if;
    end process;

    ----------------------------------------------------------------
    -- process populating the UART TX_FIFO from the FPGA TX Register
    ----------------------------------------------------------------

    process (CLK)
    begin
        if (CLK'event and CLK='1') then
            if (REG_WE='1') then
                case REG_Addr is
                    when X"2000" =>
                        TX_FIFO_DataIn <= REG_DataIn;
                        TX_FIFO_WriteEn <= '1';
                    when others =>
                end case;
            else
                TX_FIFO_WriteEn <= '0';
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------
    -- process draining the UART RX_FIFO into the FPGA's USB receive FIFO
    ---------------------------------------------------------------------

    -- DEPRECATED - switched logic over to register-based operations
    -- instead of streaming USB transfers.

    -- process (RST, CLK)
    -- begin
    --     if (RST='1') then
    --         USB_DataOut <= X"0000";
    --         USB_DataOutWE <= '0';
    --         state_RX <= state_WaitForRxFifo;
    --     elsif (CLK'event and CLK='1') then
    --         case state_RX is
    --             when state_WaitForRxFifo =>
    --                 -- clear any pending USB-to-PC write strobes
    --                 USB_DataOutWE <= '0';
    --                 if (RX_FIFO_Empty = '1') then
    --                     -- no data received yet from the UART
    --                     -- clear any pending read strobes...
    --                     RX_FIFO_ReadEn <= '0';
    --                     -- and keep waiting
    --                     state_RX <= state_WaitForRxFifo;
    --                 else
    --                     -- Oh. the RX fifo is not empty, the UART state machine
    --                     -- has placed some data in! Time to read them...
    --                     RX_FIFO_ReadEn <= '1';
    --                     -- wait one cycle for the strobe to reach the FIFO
    --                     state_RX <= state_WaitForFifoReadToStart;
    --                 end if;
    --             when state_WaitForFifoReadToStart =>
    --                 -- the RX fifo got the message, we don't want to read too much
    --                 RX_FIFO_ReadEn <= '0';
    --                 -- now wait one cycle for the RX fifo to place
    --                 -- the 8bit data in RX_FIFO_DataOut
    --                 state_RX <= state_WaitForFifoToExtract8bits;
    --             when state_WaitForFifoToExtract8bits =>
    --                 -- 8bits are out, copy them to FPGA-to-PC USB input
    --                 -- it's a 16bit bus, so copy data in the low 8 bits
    --                 USB_DataOut(7 downto 0) <= RX_FIFO_DataOut;
    --                 -- and wait one cycle for the copy to complete
    --                 state_RX <= state_WaitForUSBwriteToComplete;
    --             when state_WaitForUSBwriteToComplete =>
    --                 -- data copied to USB_DataOut!
    --                 -- now check if the FPGA-to-PC USB circuitry is busy...
    --                 if (USB_DataOutBusy = '0') then
    --                     -- it's not!
    --                     -- Strobe USB_DataOutWE to send 16bits to PC
    --                     -- (only the low 8 are used)
    --                     USB_DataOutWE <= '1';
    --                     -- then go wait for the next notificaiton about RX data
    --                     state_RX <= state_WaitForRxFifo;
    --                 else
    --                     -- FPGA-to-PC USB circuitry is busy :-(
    --                     -- wait here until it becomes free
    --                     state_RX <= state_WaitForUSBwriteToComplete;
    --                     -- USB_DataOutWE <= '0';
    --                 end if;
    --         end case;
    --     end if;
    -- end process;

    ----------------------------------------------------------------------
    -- process draining the FPGA's USB transmit FIFO into the UART TX FIFO
    ----------------------------------------------------------------------


    -- DEPRECATED - switched logic over to register-based operations
    -- instead of streaming USB transfers.


    -- tell the USB bus that we can't take any more data if...
    --    the TX FIFO is full
    -- or
    --    we are in a state where we are copying USB_DataIn to our FIFO
    --    (we have one wait state, so the USB bus must wait for us)

    -- process (RST, CLK)
    -- begin
    --     if (RST='1') then
    --         TX_FIFO_DataIn <= X"00";
    --         TX_FIFO_WriteEn <= '0';
    --         state_TX <= state_WaitForUSBTXdata;
    --     elsif (CLK'event and CLK='1') then
    --         case state_TX is
    --             when state_WaitForUSBTXdata =>
    --                 -- clear any pending TX_FIFO write strobes
    --                 TX_FIFO_WriteEn <= '0';
    --                 if (USB_DataInWE = '0') then
    --                     -- no data received yet from PC (via USB)
    --                     -- keep waiting
    --                     state_TX <= state_WaitForUSBTXdata;
    --                     USB_DataInBusy <= TX_FIFO_Full;
    --                 else
    --                     -- Oh. the PC is sending us data! 
    --                     -- Time to write them to the TX FIFO...
    --                     TX_FIFO_DataIn <= USB_DataIn(7 downto 0);
    --                     -- wait one cycle for the data to reach the FIFO
    --                     -- also block the USB bus while we copy!
    --                     -- (see USB_DataInBusy assignment above)
    --                     USB_DataInBusy <= '1';
    --                     state_TX <= state_WaitForUSBTXdataToBeCopied;
    --                 end if;
    --             when state_WaitForUSBTXdataToBeCopied =>
    --                 -- the data have been copied in the TX_FIFO_DataIn
    --                 -- write them in, strobe the fifo's WE:
    --                 TX_FIFO_WriteEn <= '1';
    --                 -- wait one cycle for the TX fifo to get them
    --                 state_TX <= state_WaitForTXFifoToWriteData;
    --             when state_WaitForTXFifoToWriteData =>
    --                 -- 8bits are now in the TX Fifo - finish the strobe
    --                 TX_FIFO_WriteEn <= '0';
    --                 -- and back to waiting
    --                 USB_DataInBusy <= '1';
    --                 state_TX <= state_WaitForUSBTXdata;
    --         end case;
    --     end if;
    -- end process;

    -------------------------------------------------------
    -- UART state machine
    ----------------------------------------
    --
    -- generator of TX processing strobe,
    -- and digital filter of serial_RX bit!
    --
    ----------------------------------------

    process (CLK, RST)
    begin
        if (RST='1') then
            baud_count_TX <= 0;
        elsif (CLK'event and CLK='1') then

            -- it takes 128 identical bits to switch the
            -- cleaned_serial_RX; otherwise keep the old value!

            filter <= filter(126 downto 0) & serial_RX;
            oldBit <= cleaned_serial_RX;
            if filter = X"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF" then
                cleaned_serial_RX <= '1';
            elsif filter = X"00000000000000000000000000000000" then
                cleaned_serial_RX <= '0';
            else
                cleaned_serial_RX <= oldBit;
            end if;

            -- Trigger the TX state machine once every cycles_per_baud
            if baud_count_TX < cycles_per_baud then
                baud_count_TX <= baud_count_TX + 1;
                -- strobe off
                s_UART_TX_step <= '0';
            else
                baud_count_TX <= 0;
                -- strobe on!
                s_UART_TX_step <= '1';
            end if;
        end if;
    end process;

    -------------------------------------------------------------
    -- UART state machines
    -------------------------------------------------------------

    process (RST, CLK)
    begin
        if (RST='1') then
            state_UART_TX <= state_Initial_or_StopBit;
            state_UART_RX <= state_Initial;
            s_readFromUART_TXFifoOnNextCycle <= '0';
            s_writeToUART_RXFifoOnNextCycle <= '0';
        elsif (CLK'event and CLK='1') then
            --
            -- First, stuff that run on EVERY cycle
            --

            -- In the previous cycle, did we ask the TX FIFO
            -- to provide us with data?
            if TX_FIFO_ReadEn = '1' then
                -- yep, we did - so complete the strobe...
                TX_FIFO_ReadEn <= '0';
                -- and wait for data to come out from the TX FIFO
                s_readFromUART_TXFifoOnNextCycle <= '1';
            end if;

            -- Are we waiting for the data to come out from the TX FIFO?
            if s_readFromUART_TXFifoOnNextCycle = '1' then
                -- yep, no need to wait any more (one cycle has passed!)
                s_readFromUART_TXFifoOnNextCycle <= '0';
                -- copy the data from the FIFO to the register we use
                -- to bit-bang the bits on the serial output
                charBeingSent <= TX_FIFO_DataOut;
            end if;

            -- In the previous cycle, did the RX FIFO store one byte of data
            -- inside RX_FIFO_DataIn?
            if s_writeToUART_RXFifoOnNextCycle = '1' then
                -- yep - so complete the strobe...
                s_writeToUART_RXFifoOnNextCycle <= '0';
                RX_FIFO_WriteEn <= '1';
            else
                RX_FIFO_WriteEn <= '0';
            end if;

            --
            -- Then, stuff that run at every BAUD cycle (e.g. at 115.2KHz)
            --
            if (s_UART_TX_step ='1') then
                case state_UART_TX is
                    when state_Initial_or_StopBit =>
                        -- Set UART output to HIGH (aka STOP bit)
                        -- in preparation for the first start bit
                        serial_TX <= '1';
                        -- Did the TX_FIFO get any incoming data from the PC?
                        if TX_FIFO_Empty = '1' then
                            -- no, keep waiting
                            state_UART_TX <= state_Initial_or_StopBit;
                        else
                            -- Yes! New data, start a strobe to get 8bits out
                            -- from the TX FIFO!
                            TX_FIFO_ReadEn <= '1';
                            -- The code running on every cycle (see above)
                            -- will fetch the 8bits into charBeingSent
                            -- by the time we reach our next state...
                            state_UART_TX <= state_StartBit;
                        end if;
                    when state_StartBit =>
                        -- set UART to LOW (aka START bit), 
                        serial_TX <= '0';
                        -- prepare for sending character data
                        bitsSentSoFar <= 0;
                        state_UART_TX <= state_SendCharacters;
                    when state_SendCharacters =>
                        -- send the LSB of charBeingSent
                        serial_TX <= charBeingSent(0);
                        -- shift charBeingSent
                        charBeingSent(6 downto 0) <= charBeingSent(7 downto 1);
                        if bitsSentSoFar = 7 then
                            -- character transmission completed, go to next stop bit
                            state_UART_TX <= state_Initial_or_StopBit;
                        end if;
                        -- keep sending bit after bit
                        bitsSentSoFar <= bitsSentSoFar + 1;
                end case;
            end if;

            -- create a pulse of s_UART_RX_step every cycles_per_baud
            if baud_count_RX < cycles_per_baud then
                baud_count_RX <= baud_count_RX + 1;
                s_UART_RX_step <= '0';
            else
                baud_count_RX <= 0;
                s_UART_RX_step <= '1';
            end if;

            case state_UART_RX is
                -- The receive part of the UART runs at full cycle speed when hunting
                -- for stop bits.
                when state_Initial =>
                    -- Did the UART input turn LOW? (aka START bit)
                    if cleaned_serial_RX = '0' then
                        -- Yes! New data are coming
                        -- reset baud alignment
                        baud_count_RX <= 0;
                        -- prepare for receiving 8 bits
                        bitsReceivedSoFar <= 0;
                        -- wait half a baud cycle before sampling the 1st bit!
                        state_UART_RX <= state_WaitHalfABaud;
                    else
                        -- Nope :-( Keep waiting for the level to drop from HIGH to LOW
                        state_UART_RX <= state_Initial;
                    end if;
                when state_WaitHalfABaud =>
                    -- Wait exactly half a baud, to sample at the center of the pulse
                    if baud_count_RX = cycles_per_baud_d2 then
                        state_UART_RX <= state_StartBitReceived;
                        -- reset baud alignment:  start counting NOW for cycles_per_baud
                        baud_count_RX <= 0;
                    else
                        state_UART_RX <= state_WaitHalfABaud;
                    end if;
                when state_StartBitReceived =>
                    -- this part runs once every cycles_per_baud
                    -- (i.e. every time s_UART_RX_step is strobed)
                    if (s_UART_RX_step ='1') then
                        charBeingRead <= cleaned_serial_RX & charBeingRead(7 downto 1);
                        if bitsReceivedSoFar = 7 then
                            -- character transmission completed, go hunt for the next stop bit
                            state_UART_RX <= state_WaitForStop;
                        end if;
                        bitsReceivedSoFar <= bitsReceivedSoFar+1;
                    end if;
                when state_WaitForStop =>
                    -- this part waits for one baud
                    -- (i.e. one s_UART_RX_step strobe)
                    if (s_UART_RX_step ='1') then
                        -- it verifies we are in STOP bit territory
                        if cleaned_serial_RX = '1' then
                            -- and stores the freshly read value in the FIFO
                            RX_FIFO_DataIn <= charBeingRead;
                            -- but waits one cycle first, for RX_FIFO_DataIn to be set
                            s_writeToUART_RXFifoOnNextCycle <= '1';
                            -- go hunt for next start bit!
                            state_UART_RX <= state_Initial;
                        else
                            -- weird condition here... assume we delayed
                            -- and landed on the start bit
                            state_UART_RX <= state_StartBitReceived;
                            bitsReceivedSoFar <= 0;
                        end if;
                    else
                        -- wait one baud period before sampling the stop bit
                        state_UART_RX <= state_WaitForStop;
                    end if;
            end case;
        end if;
    end process;

    inst_fifo_rx : CountingFifo
        PORT MAP (
            clk => CLK,
            rst => RST,
            din => RX_FIFO_DataIn,
            wr_en => RX_FIFO_WriteEn,
            rd_en => RX_FIFO_ReadEn,
            dout => RX_FIFO_DataOut,
            full => RX_FIFO_Full,
            empty => RX_FIFO_Empty,
            data_count => RX_FIFO_Count
        );

    inst_fifo_tx : CountingFifo
        PORT MAP (
            clk => CLK,
            rst => RST,
            din => TX_FIFO_DataIn,
            wr_en => TX_FIFO_WriteEn,
            rd_en => TX_FIFO_ReadEn,
            dout => TX_FIFO_DataOut,
            full => TX_FIFO_Full,
            empty => TX_FIFO_Empty,
            data_count => TX_FIFO_Count
        );

    -- Instantiate interfaces component
    Interfaces : ZestSC1_Interfaces
        port map (
            USB_StreamCLK => USB_StreamCLK,
            USB_StreamFIFOADDR => USB_StreamFIFOADDR,
            USB_StreamPKTEND_n => USB_StreamPKTEND_n,
            USB_StreamFlags_n => USB_StreamFlags_n,
            USB_StreamSLOE_n => USB_StreamSLOE_n,
            USB_StreamSLRD_n => USB_StreamSLRD_n,
            USB_StreamSLWR_n => USB_StreamSLWR_n,
            USB_StreamData => USB_StreamData,
            USB_StreamFX2Rdy => USB_StreamFX2Rdy,

            USB_RegCLK => USB_RegCLK,
            USB_RegAddr => USB_RegAddr,
            USB_RegData => USB_RegData,
            USB_RegOE_n => USB_RegOE_n,
            USB_RegRD_n => USB_RegRD_n,
            USB_RegWR_n => USB_RegWR_n,
            USB_RegCS_n => USB_RegCS_n,

            USB_Interrupt => USB_Interrupt,

            S_CLK => S_CLK,
            S_A => S_A,
            S_ADV_LD_N => S_ADV_LD_N,
            S_BWA_N => S_BWA_N,
            S_BWB_N => S_BWB_N,
            S_DA => S_DA,
            S_DB => S_DB,
            S_OE_N => S_OE_N,
            S_WE_N => S_WE_N,

            -- User connections
            -- Streaming interface
            User_CLK => CLK,
            User_RST => RST,

            User_StreamBusGrantLength => X"100", -- 256 cycles = 512 bytes

            User_StreamDataIn => USB_DataIn,
            User_StreamDataInWE => USB_DataInWE,
            User_StreamDataInBusy => USB_DataInBusy,

            User_StreamDataOut => USB_DataOut, 
            User_StreamDataOutWE => USB_DataOutWE,
            User_StreamDataOutBusy => USB_DataOutBusy,

            -- Register interface
            User_RegAddr => REG_Addr,
            User_RegDataIn => REG_DataIn,
            User_RegDataOut => REG_DataOut,
            User_RegWE => REG_WE,
            User_RegRE => REG_RE,

            -- Signals and interrupts
            User_Interrupt => '0',

            -- SRAM interface
            User_SRAM_A => "00000000000000000000000",
            User_SRAM_W => '0',
            User_SRAM_R => '0',
            User_SRAM_DR_VALID => open,
            User_SRAM_DW => "000000000000000000",
            User_SRAM_DR => open
        );

end arch;
