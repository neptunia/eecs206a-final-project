/* edge detection */
void Edge(unsigned char R[WIDTH][HEIGHT], unsigned char G[WIDTH][HEIGHT], unsigned char B[WIDTH][HEIGHT])
{
    int x = 0, y = 0, R2 = 0, G2 = 0, B2 = 0;
    unsigned char   R1[WIDTH][HEIGHT];
    unsigned char   G1[WIDTH][HEIGHT];
    unsigned char   B1[WIDTH][HEIGHT];


    /*set all the border pixels to black */
    for (y = 0; y < HEIGHT; y++) {
        for (x = 0; x < WIDTH; x++) {
            if (x == 0 || y == 0 || x == WIDTH-1 || y == HEIGHT-1){
                R[x][y] = G[x][y] = B[x][y] = 0;
            }
        }
    }

    for (y = 0; y < HEIGHT; y++) {
        for (x = 0; x < WIDTH; x++) {
            B1[x][y] = B[x][y];
            R1[x][y] = R[x][y];
            G1[x][y] = G[x][y];
        }
    }

    for (y = 1; y < HEIGHT-1; y++) {
        for (x = 1; x < WIDTH-1; x++) {
            R2 = (-1 * R1[x - 1][y + 1] - 1 * R1[x][y + 1] - 1 * R1[x + 1][y + 1] - 1 * R1[x - 1][y] + 8 * R1[x][y] - 1 * R1[x + 1][y] - 1 * R1[x - 1][y - 1] - 1 * R1[x][y - 1] - 1 * R1[x + 1][y - 1]);
            G2 = (-1 * G1[x - 1][y + 1] - 1 * G1[x][y + 1] - 1 * G1[x + 1][y + 1] - 1 * G1[x - 1][y] + 8 * G1[x][y] - 1 * G1[x + 1][y] - 1 * G1[x - 1][y - 1] - 1 * G1[x][y - 1] - 1 * G1[x + 1][y - 1]);
            B2 = (-1 * B1[x - 1][y + 1] - 1 * B1[x][y + 1] - 1 * B1[x + 1][y + 1] - 1 * B1[x - 1][y] + 8 * B1[x][y] - 1 * B1[x + 1][y] - 1 * B1[x - 1][y - 1] - 1 * B1[x][y - 1] - 1 * B1[x + 1][y - 1]);

            if (R2 < 0) {
                R[x][y] = 0;
            }
            if (R2 > 255) {
                R[x][y] = 255;
            }
            if (R2 <= 255 && R2 >= 0) {
                R[x][y] = R2;
            }

            if (G2 < 0) {
                G[x][y] = 0;
            }
            if (G2 > 255) {
                G[x][y] = 255;
            }
            if (G2 <= 255 && G2 >= 0) {
                G[x][y] = G2;
            }

            if (B2 < 0) {
                B[x][y] = 0;
            }
            if (B2 > 255) {
                B[x][y] = 255;
            }
            if (B2 <= 255 && B2 >= 0) {
                B[x][y] = B2;
            }


        }
    }

    printf("Done\n");
}