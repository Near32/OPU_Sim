#ifndef COMPORT_H
#define COMPORT_H

#include "rs232.h"
#include <iostream>
#include <string>
using namespace std;
#define LENBUF 4096

class Comport
{
    private :
    int comport_numero;
    int baudrate;
    char mode[3];
    
    
    FILE* fichier;
    
    public :
    
    char buffer[LENBUF];
    
    Comport( int comport_numero, int baudrate, const char* mode, const char* filepath)
    {
    
        /*GESTION PORT COM */
        this->comport_numero = comport_numero;
        this->baudrate = baudrate;        
        
        if(strlen(mode) != 3)
        {
            cout << "ERROR : Wrong communication mode --> default : \"8N1\" " << endl;
            //memset((this->mode),"8",8);
            this->mode[0] = '8';
            this->mode[1] = 'N';
            this->mode[2] = '1';                        
        }
        else
        {
            this->mode[0] = mode[0];
            this->mode[1] = mode[1];
            this->mode[2] = mode[2];                        
        }
        
        if(RS232_OpenComport(this->comport_numero, this->baudrate, this->mode) )
        {
            cout << "ERROR : wrong parameters" << endl;
            exit(1);
        }
        else
        {
            cout << "COMPORT opened." << endl;
        }
        
        /*----------------------------------------------------------------------------*/
        
        
        /*GESTION FICHIER ECRITURE*/
        fichier = fopen(filepath, "w+");
        if(fichier == NULL)
        {
            cout << "ERROR : cannot open the file." << endl;
            exit(1);
        }
        else
        {
            cout << "FILE opened." << endl;
        }                                
    }
    
    ~Comport()
    {
        /*COMPORT*/
        RS232_CloseComport(comport_numero);
        
        /*FILE*/
        if(fclose(fichier) == EOF)
        {
            cout << "ERROR : cannot close the file." << endl;
            exit(1);
        }
        else
        {
            cout << "FILE closed." << endl;
        }
        
    }
    
    void afficher()
    {
        int max = 512;    
        char buffer_line[max];
        int nbr_line = 0;

        while(fgets(buffer_line,max,fichier) != NULL)    // fget renvoi null si on arrive en fin de ficher...
        {
            cout << buffer_line;// pas de  "<< endl;" car fgets le stocke déjà dans le buffer.
            nbr_line++;
        }
        
        cout << "// THE FILE CONSISTS OF " << nbr_line << " LINE(S). //" << endl;
    }

    int listen()
    {                
        return RS232_PollComport(this->comport_numero, (unsigned char*)this->buffer, LENBUF);                     
    }
    
    void listenAndWrite()
    {
        if(listen() != 0)
        {            
            fputs(buffer,fichier);
            string s = string(buffer);
            //cout << s;
        }            
    }
    
    void clear()
    {
    	//this->buffer[0] = '\0' ;
    	memset(this->buffer, '\0', LENBUF);
    }
    
    void configureListening( float intervalsec)
    {
    
    
    }
    
    void mainLoop()
    {        
        while(1)
        {
            listenAndWrite();            
            
        }    
    }
};
#endif

