#ifndef PAINTING_SIDES_H
#define PAINTING_SIDES_H

// Function declarations for painting patterns
void paintSide1Pattern();
void paintSide2Pattern();
void paintSide3Pattern();
void paintSide4Pattern();

// Declaration for the combined painting sequence
void paintAllSides();

// Global variable to control the number of coats for paintAllSides
extern int g_requestedCoats;
extern int g_interCoatDelaySeconds;

#endif // PAINTING_SIDES_H 