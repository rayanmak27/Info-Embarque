#ifndef BOUTON_H
#define BOUTON_H

int bouton_init(void);
int bouton_ajouter(int gpio);
int bouton_read(int gpio);
void bouton_close(void);

#endif