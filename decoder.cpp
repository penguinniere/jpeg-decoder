#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "jpegheader.h"
using namespace std;
//define header variables
App0 app0;
Dqt dqt[4];
Sof0 sof0;
Dri dri;
Sos sos;
Trie *huffman[2][4]={0}; // dc=0 ac=1

FILE *jpegFile, *bmpFile;
int end = 0, startStream = 0;

//define data stream variables

Trie* point;
int comp, repNum, run, len, value, blockIndex;
int dcValue[3];
int size[3*2], mcuNum[2], mcuNo;
float mcu[16*16][3], block[8*8];
unsigned char *bmpData;

//define header function
void print_word( char w[], int s){ for( int i=0; i<s; i++) printf("%02X", (w[i])&0xFF);}
int wordToInt( char w[], int s){ int v=0; for( int i=0; i<s; i++){ v<<=8; v+=(int)w[i]&0xFF;} return v;}
int getLength(){ char w[2]; fread(w, sizeof(char), 2, jpegFile); return wordToInt( w, 2)-2;}

void SOI(){/*printf("SOI\n");*/}

void APP0(){
    //printf("APP0\n");
    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        memset ( &app0, 0, sizeof(App0));
        app0.size = size;
        strncpy( app0.tag, &content[0], 5);
        app0.version = wordToInt( &content[5], 2);
        app0.denseUnit = wordToInt( &content[7], 1);
        app0.xDense = wordToInt( &content[8], 2);
        app0.yDense = wordToInt( &content[10], 2);
        app0.smallXDense = wordToInt( &content[12], 1);
        app0.smallYDense = wordToInt( &content[13], 1);
        app0.smallPic = (char*) malloc(sizeof(char)*(size-14));
        strncpy( app0.smallPic, &content[14], size-14);
        
        free(content);
    }
}

void APPN( int n){
    //printf("APP%1d", n);
    
    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        
        free(content);
    }       
}

void DQT(){
    //printf("DQT\n");
    
    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        Dqt tmp;
        for( int s=0; s<size;){
            memset ( &tmp, 0, sizeof(Dqt));
            int preId = wordToInt( &content[0+s], 1);
            tmp.precision =  ((preId&0xF0)>>4);
            tmp.id = ((preId&0x0F));
            for( int i=0; i<64; i++)
                tmp.factor[i] = wordToInt( &content[s+1+i*(tmp.precision+1)], tmp.precision+1);
            /*for( int i=0; i<64; i++){
                if( i%8==0)
                    printf("\n");
                printf("%d ",tmp.factor[i]);      
            }
            printf("\nid %d precision %d\n", tmp.id, tmp.precision);*/
            memcpy( &dqt[tmp.id], &tmp, sizeof(Dqt));
            s+=64*(tmp.precision+1)+1;
        }
        free(content);
    }
}
void SOF0(){
    //printf("SOF0\n");

    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        memset( &sof0, 0, sizeof(Sof0));
        sof0.precision = wordToInt( &content[0], 1);
        sof0.size[1] = wordToInt( &content[1], 2);
        sof0.size[0] = wordToInt( &content[3], 2);
        sof0.colorComp = wordToInt( &content[5], 1);
        for( int i=0; i<sof0.colorComp; i++){
            sof0.color[4*i+0] = wordToInt( &content[6+i*3+0], 1); //id
            int sample = wordToInt( &content[6+i*3+1], 1); 
            sof0.color[4*i+1] = ((sample&0xf0)>>4); // x sample
            sof0.color[4*i+2] = (sample&0x0f); //y sample
            sof0.color[4*i+3] = wordToInt( &content[6+i*3+2], 1); //dqt id
        }
        /*printf("%d %d %d %d\n", sof0.precision, sof0.size[1], sof0.size[0], sof0.colorComp);
        for( int i=0; i<4; i++){
            for( int j=0; j<4; j++)
                printf("%d ", sof0.color[4*i+j]);
            printf("\n");
        }*/
        free(content);
    }
}
void DHT(){  //huffman tables 
    //printf("DHT\n");
    
    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        for( int i=0; i<size;){
            //AC DC ID?
            int ac = 0; // True AC, False DC 
            int id = content[i]&0x0F;
            if( ((content[i]&0xF0)>>4)==0)
                ac = 0;
            else if( ((content[i]&0xF0)>>4)==1)
                ac = 1;
            i++;
            //the number of different code size
            int count[16]={0}, countSum={0};
            for( int k=0; k<16; k++){
                count[k] = (int)(content[k+i] & 0xFF);
                countSum += count[k];
            }
            i+=16;
            //build table
            Trie *tmp = (Trie*) malloc(sizeof(Trie));
            memset( tmp, 0, sizeof(Trie));
            tmp->value=-1;
            int code = 0;
            for ( int k=0, m=0; k<16; k++, code<<=1)
                for( ; count[k]>0; count[k]--, code++, m++){
                    //printf("code %04x k %d\n", code, k);
                    Trie *point = tmp;
                    for( int h=k; h>=0; h--){
                        int bit = (code>>h)&0x01;
                        //printf("%d", bit);
                        if( point->node[bit]==NULL){
                            point->node[bit] = (Trie*) malloc(sizeof(Trie)); 
                            memset( point->node[bit], 0, sizeof(Trie));
                            point->node[bit]->value = -1;
                        }
                        point = point->node[bit];
                    }
                    //printf("\n");
                    point->value = content[i+m]&0xFF;
                    //printf("length %d code %4X content %4X\n", k+1, code, content[i+m]&0xFF);
                }
            huffman[ac][id] = tmp; //dc
            //printf("ac %d id %d\n", ac, id);
            i+=countSum;
        }
        free(content);
    }
}

void DRI(){
    //printf("DRI\n");
    
    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        dri.rst = wordToInt( &content[0], 2);
        free(content);
    }   
}

void SOS(){
    //printf("SOS\n");
    
    char *content = NULL;
    int size = getLength();
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        sos.colorComp = wordToInt( &content[0], 1);
        for( int i=0; i<sos.colorComp; i++){
            sos.color[3*i+0] = wordToInt( &content[1+2*i+0], 1);
            int dcac = wordToInt( &content[1+2*i+1], 1);
            sos.color[3*i+1] = ((dcac&0xf0)>>4);
            sos.color[3*i+2] = (dcac&0x0f);
        }
        /*
        printf("colorComp %d\n", sos.colorComp);
        for( int i=0; i<sos.colorComp; i++){
            for ( int j=0; j<3; j++)
                printf("%d ", sos.color[3*i+j]);
            printf("\n");
        }*/
        free(content);
    }
    startStream = 1;
}

void COM(){
    //printf("COM\n");
    
    char *content = NULL;
    int size = getLength();
    
    if( size > 0){
        content = (char*) malloc(sizeof(char)*size);
        fread( content, sizeof(char), size, jpegFile);
        free(content);
    }
}

void EOI(){/*printf("EOI\n");*/ end=1;}

int speciel_tag(){
    int skip = 0;
    char word;
    fread( &word, sizeof(char), 1, jpegFile);
    if( startStream){
        switch( word&0xFF){
            case 0xD9: skip = 1; EOI(); break; 
            case 0x00: break;  //use 0xFF
            case 0xD0: case 0xD1: case 0xD2: case 0xD3: case 0xD4: case 0xD5: case 0xD6: 
            case 0xD7: skip = 1; dcValue[comp]=0; break; //RSTn
            case 0xFF: default: fseek( jpegFile, -1, SEEK_CUR); skip=1; //skip fist 0xff
        }
    }else{
        switch( word&0xFF){
            case 0xD8: SOI(); break;
            case 0xE0: APP0(); break;
            case 0xE1: case 0xE2: case 0xE3: case 0xE4: case 0xE5: case 0xE6: case 0xE7: 
            case 0xE8: case 0xE9: case 0xEA: case 0xEB: case 0xEC: case 0xED: case 0xEE:
            case 0xEF: APPN( ((word-0xE0)&0xFF)); break;
            case 0xDB: DQT(); break;
            case 0xC0: SOF0(); break;
            case 0xC4: DHT(); break;
            case 0xDD: DRI(); break;
            case 0xDA: SOS(); break;
            case 0xFE: COM(); break;
        }
    }
    return skip;
}

void zig_zag_sub(float *a){
    float b[64]={0};
    for (int i=0; i<64; i++)
        b[i]=a[zig_zag_table[i]];
    memcpy( a, b, 64*sizeof(float) );    
}
/*
float idct_table[8*8];
void idct_table_build(){
    for( int u=0; u<8; u++)
        for( int x=0; x<8; x++)
            idct_table[8*u+x]=cos(((2*x+1)*u*M_PI)/16.0f);
    for( int i=0; i<8; i++)
        idct_table[8*i] /= sqrt(2.0f);
}*/

void idct(float *a){
    float b[8*8], sum;
    int y,x,u,v;
    for ( y=0; y<8; y++)
        for( x=0; x<8; x++){
            sum = 0.0;
            for( v = 0; v<8; v++)
                for( u = 0; u<8; u++)
                    sum+= a[8*v+u]*idct_table[8*u+x]*idct_table[8*v+y];
            b[8*y+x] = round(sum/4.0);
            if( !comp)
                b[8*y+x] += 128.0;
        }
    memcpy( a, b, 64*sizeof(float));
    
}

void ycbcr_to_rgb(){
    //printf("tranform\n");
    int mcuX = repNum%size[2*comp], mcuY = repNum/size[2*comp];
    int ratioX = size[0]/size[2*comp];
    int ratioY = size[1]/size[2*comp+1];
    for( int y=0; y<8; y++)
        for( int x=0; x<8; x++){
            int mX=(mcuX*8+x)*ratioX, mY=(mcuY*8+y)*ratioY;
            for( int j=0; j<size[1]/size[2*comp+1]; j++)
                for( int i=0; i<size[0]/size[2*comp]; i++){
                    //printf("%d\n",16*(mY+j)+(mX+i));
                    mcu[16*(mY+j)+(mX+i)][0] += block[y*8+x]*YCbCr[3*comp+0];  //b
                    mcu[16*(mY+j)+(mX+i)][1] += block[y*8+x]*YCbCr[3*comp+1];  //g
                    mcu[16*(mY+j)+(mX+i)][2] += block[y*8+x]*YCbCr[3*comp+2];  //r
                }
        }
    /*
    for( int i=0; i<16; i++){
        for( int j=0; j<16; j++)
            printf("%f ", mcu[16*i+j][1]);
        printf("\n");
    }*/
}

void bmp_build(){
    int X = mcuNo%mcuNum[0], Y = mcuNo/mcuNum[0];
    int baseX = 8*size[0]*X, baseY = 8*size[1]*Y;
    //printf("%d %d %d %d\n", X, Y, baseX, baseY);
    for( int j=0; j<8*size[1]; j++)
        for( int i=0; i<8*size[0]; i++)
            for( int c=0; c<3; c++){
                int value = (int) round(mcu[16*j+i][c]);
                if( value>255) value = 255;
                else if( value<0) value = 0;
                bmpData[((8*size[0]*mcuNum[0]*(baseY+j)+baseX)+i)*3+c] = (unsigned char)(value&0xFF);
                //printf("%d\n", (8*size[0]*(mcuNum[0]*(baseY+j)+baseX)+i)*3+c);
            }
    memset( mcu, 0, 16*16*3*sizeof(float));
}

void decode(){
    //dq quantify
    int dqtId = sof0.color[comp*4+3];
    for( int i=0; i<64; i++)
        block[i]*= dqt[dqtId].factor[i];
    //de_zig_zag
    zig_zag_sub( block);
    /*
    printf("\nafter zig zag\n");
    for( int i=0; i<64; i++){
        if( i%8==0)
            printf("\n");
        printf("%4.0f ", block[i]);
    }*/
    //IDCT
    idct( block);
    /*
    printf("\nafter idct\n");
    for( int i=0; i<64; i++){
        if( i%8==0)
            printf("\n");
        printf("%4.0f ", block[i]);
    }printf("\n");*/
    
    //YCbCr to RGB
    ycbcr_to_rgb();
    //reset number to next round
    repNum++;
    if( repNum==size[2*comp+0]*size[2*comp+1]){
        repNum = 0;
        comp = (comp+1)%3;
        if( !comp){ 
            bmp_build();
            mcuNo++;
            //printf("%d\n", mcuNo);
        }
    }
    memset( block, 0, sizeof(float)*64);
    point = huffman[0][sos.color[3*comp+1]]; 
    blockIndex = 0;
    //printf(" decode end\n");
}
// define writing bmp function

void build_bmp(char *path){
    //this part borrow from openstack
    bmpFile = fopen( path, "wb");
    if ( !bmpFile) {fputs ("Can't not open output file",stderr); exit (1);}
	typedef struct                       /**** BMP file header structure ****/
    {
        unsigned int   bfSize;           /* Size of file */
        unsigned short bfReserved1;      /* Reserved */
        unsigned short bfReserved2;      /* ... */
        unsigned int   bfOffBits;        /* Offset to bitmap data */
    } BITMAPFILEHEADER;

    typedef struct                       /**** BMP file info structure ****/
    {
        unsigned int   biSize;           /* Size of info header */
        int            biWidth;          /* Width of image */
        int            biHeight;         /* Height of image */
        unsigned short biPlanes;         /* Number of color planes */
        unsigned short biBitCount;       /* Number of bits per pixel */
        unsigned int   biCompression;    /* Type of compression to use */
        unsigned int   biSizeImage;      /* Size of image data */
        int            biXPelsPerMeter;  /* X pixels per meter */
        int            biYPelsPerMeter;  /* Y pixels per meter */
        unsigned int   biClrUsed;        /* Number of colors used */
        unsigned int   biClrImportant;   /* Number of important colors */
    } BITMAPINFOHEADER;

    BITMAPFILEHEADER bfh;
    BITMAPINFOHEADER bih;

    /* Magic number for file. It does not fit in the header structure due to alignment requirements, so put it outside */
    unsigned short bfType=0x4d42;           
    bfh.bfReserved1 = 0;
    bfh.bfReserved2 = 0;
    bfh.bfSize = 2+sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER)+sof0.size[0]*sof0.size[1]*3;
    bfh.bfOffBits = 0x36;

    bih.biSize = sizeof(BITMAPINFOHEADER);
    bih.biWidth = sof0.size[0];
    bih.biHeight = sof0.size[1];
    bih.biPlanes = 1;
    bih.biBitCount = 24;
    bih.biCompression = 0;
    bih.biSizeImage = 0;
    bih.biXPelsPerMeter = 5000;
    bih.biYPelsPerMeter = 5000;
    bih.biClrUsed = 0;
    bih.biClrImportant = 0;
    
    int width = ((sof0.size[0]*3)&0x0003)? ((sof0.size[0]*3)&0xFFFC)+4: ((sof0.size[0]*3)&0xFFFC); 
    int height = sof0.size[1];
    
    fwrite( &bfType,1,sizeof(bfType), bmpFile);
	fwrite( &bfh, 1, sizeof(BITMAPFILEHEADER), bmpFile);
    fwrite( &bih, 1, sizeof(BITMAPINFOHEADER), bmpFile);
    for( int j=height-1; j>=0; j--)
        fwrite( &bmpData[size[0]*8*3*mcuNum[0]*j], width, sizeof(unsigned char), bmpFile);
}

int main( int argc, char* argv[]){

    jpegFile = fopen( argv[1], "rb");
    if ( !jpegFile) {fputs ("Can't not read jpeg file",stderr); exit (1);}
    rewind(jpegFile);
    
    //do boring bonus XD
    for( int i=3; i<argc; i++){
        int coef = -1;
        float factor = 1.0;
        if( i+1<argc){ //prevent out of array
            if( !strcmp( argv[i], "-y"))
                coef = 0;  
            else if( !strcmp( argv[i], "-cb"))
                coef = 1;
            else if( !strcmp( argv[i], "-cr"))
                coef = 2;
            if( coef!=-1){
                factor = (float) atof(argv[i+1]);
                if( factor>5.0f)
                    factor = 5.0f;
                else if( factor<-5.0f)
                    factor = -5.0f;
                for( int k = 0; k < 3; k++)
                    YCbCr[3*coef+k] *= factor;
                i++;
            }
           
        }
    
    }
    char word;
    
    //finding header
    while(!startStream){
        //printf("offset %08X\n", ftell (jpegFile));
        fread( &word, sizeof(char), 1, jpegFile);
        if( (word&0xFF)==0xFF )
            speciel_tag();
    }
 
    //define data streaminformation
    for( int i=0; i<3; i++){
        size[2*i+0] = sof0.color[4*i+1];
        size[2*i+1] = sof0.color[4*i+2];
    }   
    for( int i=0; i<2; i++)
        mcuNum[i] = (int) ceil( sof0.size[i]/(size[i]*8.0));
    bmpData = (unsigned char*) malloc( sizeof(unsigned char*)*size[0]*size[1]*mcuNum[0]*mcuNum[1]*8*8*3);
    memset( bmpData, 0, sizeof(unsigned char*)*size[0]*size[1]*mcuNum[0]*mcuNum[1]*8*8*3);
    comp = 0;
    run = 0; 
    value = 0;
    blockIndex = 0;
    mcuNo = 0;
    memset( dcValue, 0, sizeof(int)*3);
    memset( block, 0, sizeof(float)*64);
    point = huffman[0][sos.color[3*comp+1]]; //1 dc 2 ac in sos
    //idct_table_build();
    //printf("%d %d %d %d\n", size[0], size[1], mcuNum[0], mcuNum[1]);
    while(!end){
        fread( &word, sizeof(char), 1, jpegFile);
        if( (word&0xFF)!=0xFF || !speciel_tag()){
            //printf( "%02X ", word&0xFF);
            for( int i=7; i>=0; i--){
                int bit = (word>>i)&0x01;
                //printf( "%d", bit);
                if( !blockIndex){ //dc
                    if(!run){
                        point = point->node[bit];
                        if( point->value==0){
                            block[blockIndex] = dcValue[comp];
                            blockIndex++;
                            point = huffman[1][sos.color[3*comp+2]];
                        }
                        else if( point->value!=-1){
                            run = point->value;
                            len = run;
                        }
                    }
                    else{
                        value<<=1;
                        value+=bit;
                        run--;
                        if(!run){
                            //printf("\nlen %d value %d\n", len, value);
                            if ( value < (1<<(len-1)))
                                 value = (-(1<<len))+1+value;
                            //printf("actual value %d\n", value);
                            block[blockIndex] = dcValue[comp] + value;
                            dcValue[comp] = block[blockIndex];
                            blockIndex++;
                            point = huffman[1][sos.color[3*comp+2]];
                            value=0;
                        }
                    }
                }
                else{ //ac
                    if(!run){
                        point = point->node[bit];
                        if( point->value==0)
                            decode();
                        else if( point->value!=-1){
                            run = ((point->value) & 0x0f);
                            len = run;
                            blockIndex += (((point->value) & 0xf0) >> 4);
                            if( run==0){
                                point = huffman[1][sos.color[3*comp+2]];
                                blockIndex++;
                                if( blockIndex>=64)
                                    decode();
                            }
                        }
                    }
                    else{
                        value<<=1;
                        value+=bit;
                        run--;
                        if(!run){
                            //printf("\nlen %d value %d\n", len, value);
                            if ( value < (1<<(len-1)))
                                 value = (-(1<<len))+1+value;
                            //printf("actual value %d\n", value);
                            block[blockIndex] = value;
                            blockIndex++;
                            point = huffman[1][sos.color[3*comp+2]];
                            value = 0;
                            if( blockIndex>=64)
                                decode();
                            
                        }
                    }
                }
            }
        //printf( "\n");
        }
    }
    
    build_bmp(argv[2]);
    return 0;
}