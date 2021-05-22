// Arduino sketch to interface between an ESP8266 and a Creed 7B teleprinter
// (c) Copyright 2021 MCQN Ltd

const int kPinTransferContacts = 5;
const int kPinInternetToTeleprinter = LED_BUILTIN;
const int kPinTransferContactIndicator = LED_BUILTIN;

bool gInputMark = false;

// For quick lookups we'll just have a 128 byte array with /all/ of the ASCII dataset in it
// That will let us map lower-case onto upper-case baudot codes, and we can define anything
// with the top bit set as an invalid baudot character that we'll ignore
// We'll just store the 5-bit code WITHOUT the start and stop bits here, as they're common
// to every code sent
// Then we can use two of the bits to show if it's in the "letters" codepage or the
// "figs" codepage or in both (CR, LF, ' ', etc. are in both
const uint8_t kInvalidCode = 0xff;
const uint8_t kLettersBaudotCode = 0b00011111;
const uint8_t kFigsBaudotCode = 0b00011011;
const uint8_t kFigsModifier    = 0b0100000;
const uint8_t kLettersModifier = 0b1000000;
const uint8_t kAsciiToBaudotTable[128] {
    /* Dec   Hex   Char                      */
    /* 0     00    NUL '\0' (null character) */ kInvalidCode,
    /* 1     01    SOH (start of heading)    */ kInvalidCode,
    /* 2     02    STX (start of text)       */ kInvalidCode,
    /* 3     03    ETX (end of text)         */ kInvalidCode,
    /* 4     04    EOT (end of transmission) */ kInvalidCode,
    /* 5     05    ENQ (enquiry)             */ kInvalidCode,  // Should this be Who Are You?
    /* 6     06    ACK (acknowledge)         */ kInvalidCode,
    /* 7     07    BEL '\a' (bell)           */ 0b00011010 | kFigsModifier,
    /* 8     08    BS  '\b' (backspace)      */ kInvalidCode,
    /* 9     09    HT  '\t' (horizontal tab) */ kInvalidCode,
    /* 10    0A    LF  '\n' (new line)       */ 0b00001000 | kFigsModifier | kLettersModifier,
    /* 11    0B    VT  '\v' (vertical tab)   */ kInvalidCode,
    /* 12    0C    FF  '\f' (form feed)      */ kInvalidCode,
    /* 13    0D    CR  '\r' (carriage ret)   */ 0b00000010 | kFigsModifier | kLettersModifier,
    /* 14    0E    SO  (shift out)           */ kInvalidCode,
    /* 15    0F    SI  (shift in)            */ kInvalidCode,
    /* 16    10    DLE (data link escape)    */ kInvalidCode,
    /* 17    11    DC1 (device control 1)    */ kInvalidCode,
    /* 18    12    DC2 (device control 2)    */ kInvalidCode,
    /* 19    13    DC3 (device control 3)    */ kInvalidCode,
    /* 20    14    DC4 (device control 4)    */ kInvalidCode,
    /* 21    15    NAK (negative ack.)       */ kInvalidCode,
    /* 22    16    SYN (synchronous idle)    */ kInvalidCode,
    /* 23    17    ETB (end of trans. blk)   */ kInvalidCode,
    /* 24    18    CAN (cancel)              */ kInvalidCode,
    /* 25    19    EM  (end of medium)       */ kInvalidCode,
    /* 26    1A    SUB (substitute)          */ kInvalidCode,
    /* 27    1B    ESC (escape)              */ kInvalidCode,
    /* 28    1C    FS  (file separator)      */ kInvalidCode,
    /* 29    1D    GS  (group separator)     */ kInvalidCode,
    /* 30    1E    RS  (record separator)    */ kInvalidCode,
    /* 31    1F    US  (unit separator)      */ kInvalidCode,
    /* 32    20    SPACE                     */ 0b00000100 | kFigsModifier | kLettersModifier,
    /* 33    21    !                         */ kInvalidCode,
    /* 34    22    "                         */ kInvalidCode,
    /* 35    23    #                         */ kInvalidCode,
    /* 36    24    $                         */ 0b00000101 | kFigsModifier, // £ really!
    /* 37    25    %                         */ 0b00010110 | kFigsModifier,
    /* 38    26    &                         */ kInvalidCode,
    /* 39    27    '                         */ 0b00010100 | kFigsModifier,
    /* 40    28    (                         */ 0b00011110 | kFigsModifier,
    /* 41    29    )                         */ 0b00001001 | kFigsModifier,
    /* 42    2A    *                         */ kInvalidCode,
    /* 43    2B    +                         */ 0b00010001 | kFigsModifier,
    /* 44    2C    ,                         */ 0b00000110 | kFigsModifier,
    /* 45    2D    -                         */ 0b00011000 | kFigsModifier,
    /* 46    2E    .                         */ 0b00000110 | kFigsModifier,
    /* 47    2F    /                         */ 0b00010111 | kFigsModifier,
    /* 48    30    0                         */ 0b00001101 | kFigsModifier,
    /* 49    31    1                         */ 0b00011101 | kFigsModifier,
    /* 50    32    2                         */ 0b00011001 | kFigsModifier,
    /* 51    33    3                         */ 0b00010000 | kFigsModifier,
    /* 52    34    4                         */ 0b00001010 | kFigsModifier,
    /* 53    35    5                         */ 0b00000001 | kFigsModifier,
    /* 54    36    6                         */ 0b00001011 | kFigsModifier,
    /* 55    37    7                         */ 0b00011100 | kFigsModifier,
    /* 56    38    8                         */ 0b00001100 | kFigsModifier,
    /* 57    39    9                         */ 0b00000011 | kFigsModifier,
    /* 58    3A    :                         */ 0b00001110 | kFigsModifier,
    /* 59    3B    ;                         */ kInvalidCode,
    /* 60    3C    <                         */ kInvalidCode,
    /* 61    3D    =                         */ 0b00001111 | kFigsModifier,
    /* 62    3E    >                         */ kInvalidCode,
    /* 63    3F    ?                         */ 0b00010011 | kFigsModifier,
    /* 64    40    @                         */ 0b00001011 | kFigsModifier,
    /* 65    41    A                         */ 0b00011000 | kLettersModifier,
    /* 66    42    B                         */ 0b00010011 | kLettersModifier,
    /* 67    43    C                         */ 0b00001110 | kLettersModifier,
    /* 68    44    D                         */ 0b00010010 | kLettersModifier,
    /* 69    45    E                         */ 0b00010000 | kLettersModifier,
    /* 70    46    F                         */ 0b00010110 | kLettersModifier,
    /* 71    47    G                         */ 0b00001011 | kLettersModifier,
    /* 72    48    H                         */ 0b00000101 | kLettersModifier,
    /* 73    49    I                         */ 0b00001100 | kLettersModifier,
    /* 74    4A    J                         */ 0b00011010 | kLettersModifier,
    /* 75    4B    K                         */ 0b00011110 | kLettersModifier,
    /* 76    4C    L                         */ 0b00001001 | kLettersModifier,
    /* 77    4D    M                         */ 0b00000111 | kLettersModifier,
    /* 78    4E    N                         */ 0b00000110 | kLettersModifier,
    /* 79    4F    O                         */ 0b00000011 | kLettersModifier,
    /* 80    50    P                         */ 0b00001101 | kLettersModifier,
    /* 81    51    Q                         */ 0b00011101 | kLettersModifier,
    /* 82    52    R                         */ 0b00001010 | kLettersModifier,
    /* 83    53    S                         */ 0b00010100 | kLettersModifier,
    /* 84    54    T                         */ 0b00000001 | kLettersModifier,
    /* 85    55    U                         */ 0b00011100 | kLettersModifier,
    /* 86    56    V                         */ 0b00001111 | kLettersModifier,
    /* 87    57    W                         */ 0b00011001 | kLettersModifier,
    /* 88    58    X                         */ 0b00010111 | kLettersModifier,
    /* 89    59    Y                         */ 0b00010101 | kLettersModifier,
    /* 90    5A    Z                         */ 0b00010001 | kLettersModifier,
    /* 91    5B    [                         */ kInvalidCode,
    /* 92    5C    \                         */ kInvalidCode,
    /* 93    5D    ]                         */ kInvalidCode,
    /* 94    5E    ^                         */ kInvalidCode,
    /* 95    5F    _                         */ kInvalidCode,
    /* 96    60    `                         */ kInvalidCode,
    /* 97    61    a                         */ 0b00011000 | kLettersModifier,
    /* 98    62    b                         */ 0b00010011 | kLettersModifier,
    /* 99    63    c                         */ 0b00001110 | kLettersModifier,
    /* 100   64    d                         */ 0b00010010 | kLettersModifier,
    /* 101   65    e                         */ 0b00010000 | kLettersModifier,
    /* 102   66    f                         */ 0b00010110 | kLettersModifier,
    /* 103   67    g                         */ 0b00001011 | kLettersModifier,
    /* 104   68    h                         */ 0b00000101 | kLettersModifier,
    /* 105   69    i                         */ 0b00001100 | kLettersModifier,
    /* 106   6A    j                         */ 0b00011010 | kLettersModifier,
    /* 107   6B    k                         */ 0b00011110 | kLettersModifier,
    /* 108   6C    l                         */ 0b00001001 | kLettersModifier,
    /* 109   6D    m                         */ 0b00000111 | kLettersModifier,
    /* 110   6E    n                         */ 0b00000110 | kLettersModifier,
    /* 111   6F    o                         */ 0b00000011 | kLettersModifier,
    /* 112   70    p                         */ 0b00001101 | kLettersModifier,
    /* 113   71    q                         */ 0b00011101 | kLettersModifier,
    /* 114   72    r                         */ 0b00001010 | kLettersModifier,
    /* 115   73    s                         */ 0b00010100 | kLettersModifier,
    /* 116   74    t                         */ 0b00000001 | kLettersModifier,
    /* 117   75    u                         */ 0b00011100 | kLettersModifier,
    /* 118   76    v                         */ 0b00001111 | kLettersModifier,
    /* 119   77    w                         */ 0b00011001 | kLettersModifier,
    /* 120   78    x                         */ 0b00010111 | kLettersModifier,
    /* 121   79    y                         */ 0b00010101 | kLettersModifier,
    /* 122   7A    z                         */ 0b00010001 | kLettersModifier,
    /* 123   7B    {                         */ kInvalidCode,
    /* 124   7C    |                         */ kInvalidCode,
    /* 125   7D    }                         */ kInvalidCode,
    /* 126   7E    ~                         */ kInvalidCode,
    /* 127   7F    DEL                       */ kInvalidCode
};

uint8_t gOutputMode = kLettersModifier;

void transmitSpace() { Serial.print("S"); digitalWrite(kPinInternetToTeleprinter, LOW); }
void transmitMark() { Serial.print("M"); digitalWrite(kPinInternetToTeleprinter, HIGH); }
// Because we need to send 1.5 stop bits, we'll define a "half-bit" pause
// and then send two when we we're sending a bit and three when sending 1.5 bits
void transmitSemiPause() { delay(1000/50*2); }

// Transmit a 5-bit baudot code
void sendCode(char aCode)
{
    // Send the start bit, which is a "space"
    transmitSpace();
    transmitSemiPause();
    transmitSemiPause();
    // Now send the five-bit code
    uint8_t bitMask = 0b10000;
    while (bitMask > 0)
    {
        if (aCode & bitMask)
        {
            transmitMark();
        }
        else
        {
            transmitSpace();
        }
        transmitSemiPause();
        transmitSemiPause();
        // Move to the next bit
        bitMask = bitMask >> 1;
    }
    // And then te 1.5 stop bits, which are a "mark"
    transmitMark();
    transmitSemiPause();
    transmitSemiPause();
    transmitSemiPause();
}

// Send a character, if it's something the teleprinter can understand
// Switches modes if need be
void sendCharacter(char aChar)
{
    // Find the code we need to send
    uint8_t baudotCode = kAsciiToBaudotTable[aChar];
    Serial.println();
    Serial.print("Mapping ");
    Serial.print(aChar);
    Serial.print(" to ");
    Serial.print(baudotCode, BIN);
    Serial.print(" ");
    if (baudotCode != kInvalidCode)
    {
        // See if we're in the right mode
        if ((baudotCode & gOutputMode) != gOutputMode)
        {
            Serial.println("Switching modes!");
            // We need to switch modes
            if (gOutputMode == kLettersModifier)
            {
                sendCode(kFigsBaudotCode);
                gOutputMode = kFigsModifier;
            }
            else
            {
                sendCode(kLettersBaudotCode);
                gOutputMode = kLettersModifier;
            }
        }
        // Now we're good to send the code we want
        sendCode(baudotCode);
    }
    // else it's not a character we can print on the teleprinter
}
  

void setup()
{
    // put your setup code here, to run once:
    pinMode(kPinTransferContacts, INPUT);
    pinMode(kPinTransferContactIndicator, OUTPUT);
    pinMode(kPinInternetToTeleprinter, OUTPUT);

    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("Teleprinter Interface");
    Serial.println("Let's go!");
#if 1
    sendCharacter('h');
    sendCharacter('e');
    sendCharacter('l');
    sendCharacter('l');
    sendCharacter('o');
    sendCharacter(' ');
    sendCharacter('w');
    sendCharacter('o');
    sendCharacter('r');
    sendCharacter('l');
    sendCharacter('d');
    sendCharacter('?');
    while(1) { delay(100); };
#endif
}

void loop()
{
    if (digitalRead(kPinTransferContacts) != gInputMark) {
        // It's changed state
        gInputMark = !gInputMark;
        Serial.print(gInputMark);
        digitalWrite(kPinTransferContactIndicator, gInputMark);
    }
    delay(10);
}
