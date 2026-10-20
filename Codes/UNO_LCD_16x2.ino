/*
   ============================================================
   ARDUINO UNO - USER SCREEN INTERACTION CODE
   ============================================================

   RESPONSIBILITY OF UNO:
   ----------------------
   1. 16x2 I2C LCD display
   2. KY-040 rotary encoder
   3. KY-040 push button = SELECT / ENTER
   4. External BACK button
   5. Main menu
   6. Stretch menu
   7. X/Y stretch value entry
   8. Communication with Arduino Mega
   9. Displaying Mega's status to the user

   UNO DOES NOT:
   --------------
   - Drive motors
   - Read motor encoders
   - Read limit switches
   - Perform motor positioning

   Those jobs belong to the Arduino Mega.
*/


// ============================================================
// LIBRARIES
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// ============================================================
// 16x2 I2C LCD SETTINGS
// ============================================================

#define LCD_ADDRESS 0x27  // Change to 0x3F if required by your LCD backpack.
#define LCD_COLUMNS 16
#define LCD_ROWS 2

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// ============================================================
// KY-040 PINS
// ============================================================

#define ENC_CLK 2
#define ENC_DT  3
#define ENC_SW  4


// ============================================================
// BACK BUTTON
// ============================================================

#define BACK_BUTTON 5


// ============================================================
// MENU SETTINGS
// ============================================================

// The KY-040 normally has approximately 20 detents/revolution.
//
// 1/6 revolution = approximately 60 degrees.
//
// Since 20/6 = 3.33 detents, an exact 1/6 rotation cannot
// be represented physically by the detented KY-040.
//
// We therefore use 3 detents for one menu movement.
//
// This can easily be changed later after physical testing.
const int MENU_DETENTS = 3;


// ============================================================
// STRETCH VALUE SETTINGS
// ============================================================

// Minimum allowed stretch.
const float MIN_STRETCH = 1.00;

// Maximum allowed stretch.
const float MAX_STRETCH = 5.00;

// Numerical least count.
const float STRETCH_STEP = 0.01;


// ============================================================
// TIMING
// ============================================================

const unsigned long WELCOME_TIME = 3000;

const unsigned long MESSAGE_TIME = 3000;

// Button debounce time.
const unsigned long BUTTON_DEBOUNCE = 200;


// ============================================================
// PAGES
// ============================================================

enum Page
{
  MAIN_MENU_PAGE,
  STRETCH_MENU_PAGE,
  X_EDIT_PAGE,
  Y_EDIT_PAGE,
  PROCESSING_PAGE
};

Page currentPage = MAIN_MENU_PAGE;


// ============================================================
// MAIN MENU OPTIONS
// ============================================================

enum MainOption
{
  MAIN_STRETCH = 0,
  MAIN_HOME = 1
};

int mainCursor = MAIN_STRETCH;


// ============================================================
// STRETCH MENU OPTIONS
// ============================================================

enum StretchOption
{
  OPTION_X = 0,
  OPTION_Y = 1,
  OPTION_PROCEED = 2
};

int stretchCursor = OPTION_X;


// ============================================================
// X/Y STRETCH VALUES
// ============================================================

// These are the actual saved values.
float xStretch = 1.00;
float yStretch = 1.00;


// Temporary editing values.
//
// These are used so that pressing BACK during editing does
// NOT save the changed value.
float xEditValue = 1.00;
float yEditValue = 1.00;


// ============================================================
// ROTARY ENCODER VARIABLES
// ============================================================

int lastCLK = HIGH;

// Number of encoder detents accumulated.
int encoderDetents = 0;


// ============================================================
// BUTTON TIMING
// ============================================================

unsigned long lastSelectTime = 0;
unsigned long lastBackTime = 0;


// ============================================================
// COMMUNICATION WITH MEGA
// ============================================================

/*
   The UNO uses its hardware Serial port to communicate with
   the MEGA.

   IMPORTANT:
   On an UNO, hardware Serial is also connected to USB.

   Therefore, for the final physical machine, we will need
   to decide whether:
      - USB is disconnected during operation, or
      - we use a SoftwareSerial connection instead.

   For the first communication test, we use Serial.

   Communication format:

   UNO → MEGA

   CHECK_HOME
   HOME
   STRETCH 3.56 2.35

   MEGA → UNO

   HOME_ALREADY
   HOME_INCOMPLETE
   HOME_PROCESSING
   HOME_COMPLETE
   STRETCH_PROCESSING
   STRETCH_COMPLETE
   MOTION_ERROR
*/


String megaMessage = "";

// True only while waiting for the response to CHECK_HOME.
// This distinguishes a home-position check from the HOME procedure.
bool waitingForHomeCheck = false;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void showWelcome();
void showMainMenu();
void showStretchMenu();

void updateEncoder();
bool selectPressed();
bool backPressed();

void handleMainMenu();
void handleStretchMenu();
void handleXEdit();
void handleYEdit();

void sendToMega(String command);
void readMegaMessages();
void processMegaMessage(String message);

void requestHomeCheck();
void requestHome();
void requestStretch();

void showTemporaryMessage(String line1, String line2);

void resetToMainMenu();



// ============================================================
// SETUP
// ============================================================

void setup()
{
  // Start serial communication with Mega.
  Serial.begin(115200);


  // ----------------------------------------------------------
  // KY-040
  // ----------------------------------------------------------

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);


  // ----------------------------------------------------------
  // BACK BUTTON
  // ----------------------------------------------------------

  pinMode(BACK_BUTTON, INPUT_PULLUP);


  // ----------------------------------------------------------
  // 16x2 I2C LCD
  // ----------------------------------------------------------

  lcd.init();
  lcd.backlight();
  lcd.clear();


  // Initial encoder state.
  lastCLK = digitalRead(ENC_CLK);


  // ----------------------------------------------------------
  // WELCOME SCREEN
  // ----------------------------------------------------------

  showWelcome();

  delay(WELCOME_TIME);


  // ----------------------------------------------------------
  // MAIN MENU
  // ----------------------------------------------------------

  resetToMainMenu();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // Read rotary encoder.
  updateEncoder();


  // Check whether Mega has sent anything.
  readMegaMessages();


  // Handle whichever page is currently active.

  switch (currentPage)
  {
    case MAIN_MENU_PAGE:
      handleMainMenu();
      break;

    case STRETCH_MENU_PAGE:
      handleStretchMenu();
      break;

    case X_EDIT_PAGE:
      handleXEdit();
      break;

    case Y_EDIT_PAGE:
      handleYEdit();
      break;

    case PROCESSING_PAGE:
      // During processing, Mega is controlling the mechanism.
      // We continue reading Serial messages in loop().
      break;
  }
}


// ============================================================
// LCD DISPLAY HELPERS
// ============================================================

// Print one LCD row and clear any leftover characters from the
// previous contents. Character LCD rows are limited to 16 chars.
void printLCDLine(byte row, String text)
{
  if (text.length() > LCD_COLUMNS)
  {
    text = text.substring(0, LCD_COLUMNS);
  }

  lcd.setCursor(0, row);
  lcd.print("                ");
  lcd.setCursor(0, row);
  lcd.print(text);
}


// ============================================================
// WELCOME SCREEN
// ============================================================

void showWelcome()
{
  lcd.clear();
  printLCDLine(0, "Welcome!");
  printLCDLine(1, "Initializing...");
}


// ============================================================
// MAIN MENU DISPLAY
// ============================================================

void showMainMenu()
{
  printLCDLine(0, "MAIN MENU :");

  if (mainCursor == MAIN_STRETCH)
  {
    printLCDLine(1, ">STRETCH    HOME");
  }
  else
  {
    printLCDLine(1, "STRETCH   >HOME");
  }
}


// ============================================================
// STRETCH MENU DISPLAY
// ============================================================

void showStretchMenu()
{
  // Keep the heading and GO option on the first line.
  if (stretchCursor == OPTION_PROCEED)
  {
    printLCDLine(0, "STRETCH :  >GO");
  }
  else
  {
    printLCDLine(0, "STRETCH :   GO");
  }

  // Compact format keeps both saved stretch factors visible.
  // The cursor is static; there is no blinking.
  String line2;

  if (stretchCursor == OPTION_X)
  {
    line2 = ">X" + String(xStretch, 2) + "x Y" + String(yStretch, 2) + "x";
  }
  else if (stretchCursor == OPTION_Y)
  {
    line2 = "X" + String(xStretch, 2) + "x >Y" + String(yStretch, 2) + "x";
  }
  else
  {
    line2 = "X" + String(xStretch, 2) + "x Y" + String(yStretch, 2) + "x";
  }

  printLCDLine(1, line2);
}



// ============================================================
// ROTARY ENCODER READING
// ============================================================

void updateEncoder()
{
  int currentCLK = digitalRead(ENC_CLK);


  // Detect a change on CLK.
  if (currentCLK != lastCLK)
  {
    // We only count one edge of the quadrature signal.
    if (currentCLK == LOW)
    {
      int dtState = digitalRead(ENC_DT);


      if (dtState != currentCLK)
      {
        // Clockwise.
        encoderDetents++;
      }
      else
      {
        // Counter-clockwise.
        encoderDetents--;
      }
    }


    lastCLK = currentCLK;
  }
}


// ============================================================
// KY-040 PUSH BUTTON
// ============================================================

bool selectPressed()
{
  if (digitalRead(ENC_SW) == LOW)
  {
    if (millis() - lastSelectTime > BUTTON_DEBOUNCE)
    {
      lastSelectTime = millis();

      return true;
    }
  }

  return false;
}


// ============================================================
// BACK BUTTON
// ============================================================

bool backPressed()
{
  if (digitalRead(BACK_BUTTON) == LOW)
  {
    if (millis() - lastBackTime > BUTTON_DEBOUNCE)
    {
      lastBackTime = millis();

      return true;
    }
  }

  return false;
}


// ============================================================
// MAIN MENU HANDLER
// ============================================================

void handleMainMenu()
{
  // ----------------------------------------------------------
  // MENU ROTATION
  // ----------------------------------------------------------

  if (abs(encoderDetents) >= MENU_DETENTS)
  {
    if (encoderDetents > 0)
    {
      // CW → next option.
      mainCursor++;

      if (mainCursor > MAIN_HOME)
      {
        mainCursor = MAIN_STRETCH;
      }
    }
    else
    {
      // CCW → previous option.
      mainCursor--;

      if (mainCursor < MAIN_STRETCH)
      {
        mainCursor = MAIN_HOME;
      }
    }


    // Clear movement after one menu movement.
    encoderDetents = 0;


    // Redraw with the cursor in its new position.


    showMainMenu();
  }


  // ----------------------------------------------------------
  // SELECT
  // ----------------------------------------------------------

  if (selectPressed())
  {
    // ========================================================
    // STRETCH SELECTED
    // ========================================================

    if (mainCursor == MAIN_STRETCH)
    {
      // Ask Mega to check the three limit switches.
      requestHomeCheck();

      // We don't immediately change page.
      // We wait for Mega's response.
    }


    // ========================================================
    // HOME SELECTED
    // ========================================================

    else if (mainCursor == MAIN_HOME)
    {
      // Ask Mega to check HOME and perform homing if required.
      requestHome();
    }
  }


  // Back does nothing on Main Menu.
}


// ============================================================
// STRETCH MENU HANDLER
// ============================================================

void handleStretchMenu()
{
  // ----------------------------------------------------------
  // BACK
  // ----------------------------------------------------------

  if (backPressed())
  {
    resetToMainMenu();

    return;
  }


  // ----------------------------------------------------------
  // ROTARY ENCODER
  // ----------------------------------------------------------

  if (abs(encoderDetents) >= MENU_DETENTS)
  {
    if (encoderDetents > 0)
    {
      // CW.
      stretchCursor++;

      if (stretchCursor > OPTION_PROCEED)
      {
        stretchCursor = OPTION_X;
      }
    }
    else
    {
      // CCW.
      stretchCursor--;

      if (stretchCursor < OPTION_X)
      {
        stretchCursor = OPTION_PROCEED;
      }
    }


    encoderDetents = 0;

    showStretchMenu();
  }


  // ----------------------------------------------------------
  // SELECT
  // ----------------------------------------------------------

  if (selectPressed())
  {
    // X option.
    if (stretchCursor == OPTION_X)
    {
      xEditValue = xStretch;

      currentPage = X_EDIT_PAGE;

      encoderDetents = 0;
    }


    // Y option.
    else if (stretchCursor == OPTION_Y)
    {
      yEditValue = yStretch;

      currentPage = Y_EDIT_PAGE;

      encoderDetents = 0;
    }


    // Proceed.
    else if (stretchCursor == OPTION_PROCEED)
    {
      requestStretch();

      return;
    }
  }


  // The display is refreshed only when the cursor moves or
  // when returning from an edit page, avoiding repeated writes.
}


// ============================================================
// X VALUE EDITING
// ============================================================

void handleXEdit()
{
  static float lastShownX = -1.0;
  static bool xScreenDrawn = false;

  // ----------------------------------------------------------
  // BACK
  // ----------------------------------------------------------

  if (backPressed())
  {
    // Do NOT save xEditValue.

    currentPage = STRETCH_MENU_PAGE;

    encoderDetents = 0;
    xScreenDrawn = false;

    showStretchMenu();

    return;
  }


  // ----------------------------------------------------------
  // VALUE CHANGE
  // ----------------------------------------------------------

  while (encoderDetents != 0)
  {
    if (encoderDetents > 0)
    {
      // CW → increase.
      xEditValue += STRETCH_STEP;

      if (xEditValue > MAX_STRETCH)
      {
        xEditValue = MIN_STRETCH;
      }

      encoderDetents--;
    }
    else
    {
      // CCW → decrease.
      xEditValue -= STRETCH_STEP;

      if (xEditValue < MIN_STRETCH)
      {
        xEditValue = MAX_STRETCH;
      }

      encoderDetents++;
    }
  }


  // ----------------------------------------------------------
  // ACCEPT
  // ----------------------------------------------------------

  if (selectPressed())
  {
    // Save temporary value.
    xStretch = xEditValue;

    currentPage = STRETCH_MENU_PAGE;

    encoderDetents = 0;
    xScreenDrawn = false;

    showStretchMenu();

    return;
  }


  // ----------------------------------------------------------
  // DISPLAY
  // ----------------------------------------------------------

  if (!xScreenDrawn || lastShownX != xEditValue)
  {
    printLCDLine(0, "SET X STRETCH");
    printLCDLine(1, String(xEditValue, 2) + "x CW:+ CCW:-");
    lastShownX = xEditValue;
    xScreenDrawn = true;
  }
}


// ============================================================
// Y VALUE EDITING
// ============================================================

void handleYEdit()
{
  static float lastShownY = -1.0;
  static bool yScreenDrawn = false;

  // ----------------------------------------------------------
  // BACK
  // ----------------------------------------------------------

  if (backPressed())
  {
    // Do NOT save yEditValue.

    currentPage = STRETCH_MENU_PAGE;

    encoderDetents = 0;
    yScreenDrawn = false;

    showStretchMenu();

    return;
  }


  // ----------------------------------------------------------
  // VALUE CHANGE
  // ----------------------------------------------------------

  while (encoderDetents != 0)
  {
    if (encoderDetents > 0)
    {
      // CW → increase.
      yEditValue += STRETCH_STEP;

      if (yEditValue > MAX_STRETCH)
      {
        yEditValue = MIN_STRETCH;
      }

      encoderDetents--;
    }
    else
    {
      // CCW → decrease.
      yEditValue -= STRETCH_STEP;

      if (yEditValue < MIN_STRETCH)
      {
        yEditValue = MAX_STRETCH;
      }

      encoderDetents++;
    }
  }


  // ----------------------------------------------------------
  // ACCEPT
  // ----------------------------------------------------------

  if (selectPressed())
  {
    yStretch = yEditValue;

    currentPage = STRETCH_MENU_PAGE;

    encoderDetents = 0;
    yScreenDrawn = false;

    showStretchMenu();

    return;
  }


  // ----------------------------------------------------------
  // DISPLAY
  // ----------------------------------------------------------

  if (!yScreenDrawn || lastShownY != yEditValue)
  {
    printLCDLine(0, "SET Y STRETCH");
    printLCDLine(1, String(yEditValue, 2) + "x CW:+ CCW:-");
    lastShownY = yEditValue;
    yScreenDrawn = true;
  }
}


// ============================================================
// SEND COMMAND TO MEGA
// ============================================================

void sendToMega(String command)
{
  Serial.println(command);

  // Also print to PC Serial Monitor for debugging.
  Serial.print("UNO -> MEGA: ");
  Serial.println(command);
}


// ============================================================
// ASK MEGA TO CHECK HOME
// ============================================================

void requestHomeCheck()
{
  // Mark that the next HOME_COMPLETE/HOME_INCOMPLETE response
  // belongs to the STRETCH menu's home-position check.
  waitingForHomeCheck = true;

  sendToMega("CHECK_HOME");

  // We wait for:
  //
  // HOME_COMPLETE
  // or
  // HOME_INCOMPLETE
}


// ============================================================
// ASK MEGA TO HOME
// ============================================================

void requestHome()
{
  sendToMega("HOME");

  // Mega will reply:
  //
  // HOME_ALREADY
  //
  // or
  //
  // HOME_PROCESSING
  //
  // followed later by
  //
  // HOME_COMPLETE
}


// ============================================================
// ASK MEGA TO PERFORM STRETCH
// ============================================================

void requestStretch()
{
  // Send both numerical values to Mega.
  //
  // Example:
  //
  // STRETCH 3.56 2.35

  Serial.print("STRETCH ");
  Serial.print(xStretch, 2);
  Serial.print(" ");
  Serial.println(yStretch, 2);


  // Show processing page immediately.
  currentPage = PROCESSING_PAGE;

  lcd.clear();
  printLCDLine(0, "Stretching...");
  printLCDLine(1, "Please wait");
}


// ============================================================
// READ MESSAGES FROM MEGA
// ============================================================

void readMegaMessages()
{
  while (Serial.available() > 0)
  {
    char receivedCharacter = Serial.read();


    // A newline means the Mega has finished sending
    // one complete message.

    if (receivedCharacter == '\n')
    {
      megaMessage.trim();

      if (megaMessage.length() > 0)
      {
        processMegaMessage(megaMessage);
      }

      megaMessage = "";
    }
    else
    {
      megaMessage += receivedCharacter;
    }
  }
}


// ============================================================
// PROCESS MEGA MESSAGE
// ============================================================

void processMegaMessage(String message)
{
  Serial.print("MEGA -> UNO: ");
  Serial.println(message);


  // ========================================================
  // HOME ALREADY
  // ========================================================

  if (message == "HOME_ALREADY")
  {
    showTemporaryMessage(
      "Already at HOME",
      "No action needed"
    );

    resetToMainMenu();
  }


  // ========================================================
  // HOME INCOMPLETE
  // ========================================================

  else if (message == "HOME_INCOMPLETE")
  {
    // The home check failed; do not open the STRETCH menu.
    waitingForHomeCheck = false;

    showTemporaryMessage(
      "Not at HOME",
      "Return HOME first"
    );

    resetToMainMenu();
  }


  // ========================================================
  // HOME PROCESSING
  // ========================================================

  else if (message == "HOME_PROCESSING")
  {
    lcd.clear();
    printLCDLine(0, "Moving to HOME...");
    printLCDLine(1, "Please wait");

    currentPage = PROCESSING_PAGE;
  }


  // ========================================================
  // HOME COMPLETE
  // ========================================================

  else if (message == "HOME_COMPLETE")
  {
    // If this response came from CHECK_HOME, the mechanism is
    // already at home, so proceed directly to the STRETCH menu.
    if (waitingForHomeCheck)
    {
      waitingForHomeCheck = false;

      currentPage = STRETCH_MENU_PAGE;
      stretchCursor = OPTION_X;
      encoderDetents = 0;

      showStretchMenu();
    }
    else
    {
      // Preserve the original behaviour when a HOME operation
      // has finished: show the message and return to Main Menu.
      showTemporaryMessage(
        "Returned to HOME",
        "Position restored"
      );

      resetToMainMenu();
    }
  }


  // ========================================================
  // STRETCH PROCESSING
  // ========================================================

  else if (message == "STRETCH_PROCESSING")
  {
    currentPage = PROCESSING_PAGE;

    lcd.clear();
    printLCDLine(0, "Stretching...");
    printLCDLine(1, "Please wait");
  }


  // ========================================================
  // STRETCH COMPLETE
  // ========================================================

  else if (message == "STRETCH_COMPLETE")
  {
    showTemporaryMessage(
      "Stretch complete",
      ""
    );

    resetToMainMenu();
  }


  // ========================================================
  // MOTION ERROR
  // ========================================================

  else if (message == "MOTION_ERROR")
  {
    showTemporaryMessage(
      "MOTION ERROR",
      "Check mechanism"
    );

    resetToMainMenu();
  }
}


// ============================================================
// TEMPORARY MESSAGE
// ============================================================

void showTemporaryMessage(
  String line1,
  String line2)
{
  lcd.clear();
  printLCDLine(0, line1);
  printLCDLine(1, line2);

  // Preserve the existing message display duration.
  delay(MESSAGE_TIME);
}



// ============================================================
// RESET UI TO MAIN MENU
// ============================================================

void resetToMainMenu()
{
  currentPage = MAIN_MENU_PAGE;

  // Main menu always starts at STRETCH.
  mainCursor = MAIN_STRETCH;

  // Stretch menu resets its cursor.
  stretchCursor = OPTION_X;

  // Clear encoder movement.
  encoderDetents = 0;

  // Main-menu cursor starts on STRETCH.

  showMainMenu();
}