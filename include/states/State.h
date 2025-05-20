#ifndef STATE_H
#define STATE_H

class State {
public:
    virtual ~State() {}
    virtual void enter() = 0;
    virtual void update() = 0;
    virtual void exit() = 0;
    virtual const char* getName() const = 0;
};

#endif // STATE_H 