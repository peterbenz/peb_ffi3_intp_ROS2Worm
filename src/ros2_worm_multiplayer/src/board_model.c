// A simple variant of the game Snake
//
// Used for teaching in classes
//
// Author:
// Franz Regensburger
// Ingolstadt University of Applied Sciences
// (C) 2011
//
// The board model

#include <curses.h>
#include <time.h>
#include <stdlib.h>
#include "worm.h"
#include "board_model.h"
#include "messages.h"

// Place an item onto the curses display.
void placeItem(struct board* aboard, int y, int x, enum BoardCodes board_code,
chtype symbol, enum ColorPairs color_pair) {

    //  Store item on the display (symbol code)
    move(y, x);                         // Move cursor to (y,x)
    attron(COLOR_PAIR(color_pair));     // Start writing in selected color
    addch(symbol);                      // Store symbol on the virtual display
    attroff(COLOR_PAIR(color_pair));    // Stop writing in selected color

    aboard->cells[y][x] = board_code;  // add the symbol to the board struct
}

// Place food item randomly and avoid collision with other objects when placing
// bonus may only be 1, 2 or 3!
void placeFood(struct board* aboard, int bonus) {
  // Local variables
  enum BoardCodes bonus_board_code;
  chtype bonus_symbol;
  enum ColorPairs bonus_color_pair;
  struct pos bonus_pos;
 
  // Set seed for random number generation
  srand(time(NULL));

  // Set local variables according to given bonus
  switch (bonus) {
    case 1:
      bonus_board_code = BC_FOOD_1;
      bonus_symbol = SYMBOL_FOOD_1;
      bonus_color_pair = COLP_FOOD_1;
      break;
    case 2:
      bonus_board_code = BC_FOOD_2;
      bonus_symbol = SYMBOL_FOOD_2;
      bonus_color_pair = COLP_FOOD_2;
      break;
    case 3:
      bonus_board_code = BC_FOOD_3;
      bonus_symbol = SYMBOL_FOOD_3;
      bonus_color_pair = COLP_FOOD_3;
      break;
    default:
      // THIS SHOULD NEVER HAPPEN!
      return;
  }
  
  do {
    bonus_pos.y = rand() % MIN_NUMBER_OF_ROWS;
    bonus_pos.x = rand() % MIN_NUMBER_OF_COLS;
  } while (getContentAt(aboard, bonus_pos) != BC_FREE_CELL);

  placeItem(aboard,
    bonus_pos.y, bonus_pos.x,
    bonus_board_code,
    bonus_symbol,
    bonus_color_pair
  );

  // Increment number of food items
  // Attention: must match number of items placed on the board above
  aboard->food_items++;
}

// Initialize the board
enum ResCodes initializeLevel(struct board * aboard) {
  int x, y, y_end;  // define local variables for loops etc
  
  // Fill board and screen buffer with empty cells.
  for (y = 0; y <= MIN_NUMBER_OF_ROWS ; y++) {
    for (x = 0; x <= MIN_NUMBER_OF_COLS ; x++) {
      placeItem(aboard,y,x,BC_FREE_CELL,SYMBOL_FREE_CELL,COLP_FREE_CELL);
    }
  }
  // Draw a line in order to separate the message area
  // Note: we cannot use function placeItem() since the message area
  // is outside the board!
  y = aboard->last_row + 1;
  for (x=0; x <= MIN_NUMBER_OF_COLS; x++) {
    move(y, x);
    attron(COLOR_PAIR(COLP_BARRIER));
    addch(SYMBOL_BARRIER);
    attroff(COLOR_PAIR(COLP_BARRIER));
  }
  // Draw a line to signal the rightmost column of the board.
  x = aboard->last_col + 1;
  for (y=0; y <= aboard->last_row ; y++) {
    move(y, x);
    attron(COLOR_PAIR(COLP_BARRIER));
    addch(SYMBOL_BARRIER);
    attroff(COLOR_PAIR(COLP_BARRIER));
  }

  // Set seed for random number generation
  srand(time(NULL));

  // Barriers: use a loop
  y = rand() % MIN_NUMBER_OF_ROWS;  // Determine position randomly
  y_end = (y + 15) % MIN_NUMBER_OF_ROWS;  // Wrap around the playing field if necessary
  for (; y != y_end; y = (y + 1) % MIN_NUMBER_OF_ROWS) {  // Note: variable for iteration already set
    x = 22;
    placeItem(aboard,y,x,BC_BARRIER,SYMBOL_BARRIER,COLP_BARRIER);
  }

  y = rand() % MIN_NUMBER_OF_ROWS;  // Determine position randomly
  y_end = (y + 15) % MIN_NUMBER_OF_ROWS;  // Wrap around the playing field if necessary
  for (; y != y_end; y = (y + 1) % MIN_NUMBER_OF_ROWS) {  // Note: variable for iteration already set
    x = 46;
    placeItem(aboard,y,x,BC_BARRIER,SYMBOL_BARRIER,COLP_BARRIER);
  }

  // Initialize number of food items
  aboard->food_items = 0;

  // Place foot items
  placeFood(aboard, 1);
  placeFood(aboard, 1);
  placeFood(aboard, 1); 
  placeFood(aboard, 2);
  placeFood(aboard, 2);
  placeFood(aboard, 2);
  placeFood(aboard, 3);
  placeFood(aboard, 3);
  placeFood(aboard, 3);
  placeFood(aboard, 3);
  
  return RES_OK;
}

enum ResCodes initializeBoard(struct board * aboard) {
  // Check dimensions of the board
  if ( COLS < MIN_NUMBER_OF_COLS ||
    LINES < MIN_NUMBER_OF_ROWS + ROWS_RESERVED ) {
    char buf[100];
    sprintf(buf,"Das Fenster ist zu klein: wir brauchen %dx%d",
    MIN_NUMBER_OF_COLS , MIN_NUMBER_OF_ROWS + ROWS_RESERVED );
    showDialog(buf,"Bitte eine Taste druecken");
    return RES_FAILED;
  }
  // Maximal index of a row
  aboard->last_row = MIN_NUMBER_OF_ROWS - 1;
  // Maximal index of a column
  aboard->last_col = MIN_NUMBER_OF_COLS - 1;
  return RES_OK;
}

// Getters

// Get the last usable row on the display
int getLastRowOnBoard(struct board* aboard) {
  return aboard->last_row;
}

// Get the last usable column on the display
int getLastColOnBoard(struct board* aboard) {
  return aboard->last_col;
}

// Get the number of food items on the board
int getNumberOfFoodItems(struct board* aboard) {
  return aboard->food_items;
}

// Get content of board at given position
enum BoardCodes getContentAt(struct board* aboard, struct pos position) {
  return aboard->cells[position.y][position.x];
}

// Setters

// Set how many food items to display on the board
void setNumberOFoodItems(struct board* aboard, int n) {
  aboard->food_items = n;
}

// Decrement the number of food items
void decrementNumberOfFoodItems(struct board* aboard) {
  aboard->food_items -= 1;  // implementation using -= operator
}
