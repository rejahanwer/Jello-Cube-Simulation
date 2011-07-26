#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of integrators
// update the jello structure accordingly
void Euler(struct world * jello);
void RK4  (struct world * jello);

#endif

