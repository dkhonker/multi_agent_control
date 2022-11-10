if ~pis_init 
            thpx1.X = time; thpx1.Y = true_p1(1);
            ehpx1.X = time; ehpx1.Y = des_p1(1);
            thpy1.X = time; thpy1.Y = true_p1(2);
            ehpy1.X = time; ehpy1.Y = des_p1(2);
            thpz1.X = time; thpz1.Y = true_p1(3);
            ehpz1.X = time; ehpz1.Y = des_p1(3);
            
            thpx2.X = time; thpx2.Y = true_p2(1);
            ehpx2.X = time; ehpx2.Y = des_p2(1);
            thpy2.X = time; thpy2.Y = true_p2(2);
            ehpy2.X = time; ehpy2.Y = des_p2(2);
            thpz2.X = time; thpz2.Y = true_p2(3);
            ehpz2.X = time; ehpz2.Y = des_p2(3);
            
            thpx3.X = time; thpx3.Y = true_p3(1);
            ehpx3.X = time; ehpx3.Y = des_p3(1);
            thpy3.X = time; thpy3.Y = true_p3(2);
            ehpy3.X = time; ehpy3.Y = des_p3(12);
            thpz3.X = time; thpz3.Y = true_p3(3);
            ehpz3.X = time; ehpz3.Y = des_p3(3);
            
            thpx4.X = time; thpx4.Y = true_p4(1);
            ehpx4.X = time; ehpx4.Y = des_p4(1);
            thpy4.X = time; thpy4.Y = true_p4(2);
            ehpy4.X = time; ehpy4.Y = des_p4(2);
            thpz4.X = time; thpz4.Y = true_p4(3);
            ehpz4.X = time; ehpz4.Y = des_p4(3);
            
            thpx5.X = time; thpx5.Y = true_p5(1);
            ehpx5.X = time; ehpx5.Y = des_p5(1);
            thpy5.X = time; thpy5.Y = true_p5(2);
            ehpy5.X = time; ehpy5.Y = des_p5(2);
            thpz5.X = time; thpz5.Y = true_p5(3);
            ehpz5.X = time; ehpz5.Y = des_p5(3);
            
            thpx6.X = time; thpx6.Y = true_p6(1);
            ehpx6.X = time; ehpx6.Y = des_p6(1);
            thpy6.X = time; thpy6.Y = true_p6(2);
            ehpy6.X = time; ehpy6.Y = des_p6(2);
            thpz6.X = time; thpz6.Y = true_p6(3);
            ehpz6.X = time; ehpz6.Y = des_p6(3);
        else
            thpx1.X = [thpx1.X, time]; thpx1.Y = [thpx1.Y, true_p1(1)];
            ehpx1.X = [ehpx1.X, time]; ehpx1.Y = [ehpx1.Y, des_p1(1)];
            thpy1.X = [thpy1.X, time]; thpy1.Y = [thpx1.Y, true_p1(2)];
            ehpy1.X = [ehpy1.X, time]; ehpy1.Y = [ehpx1.Y, des_p1(2)];
            thpz1.X = [thpz1.X, time]; thpz1.Y = [thpx1.Y, true_p1(3)];
            ehpz1.X = [ehpz1.X, time]; ehpz1.Y = [ehpx1.Y, des_p1(3)];
     
            thpx2.X = [thpx2.X, time]; thpx2.Y = [thpx2.Y, true_p2(1)];
            ehpx2.X = [ehpx2.X, time]; ehpx2.Y = [ehpx2.Y, des_p2(1)];
            thpy2.X = [thpy2.X, time]; thpy2.Y = [thpy2.Y, true_p2(2)];
            ehpy2.X = [ehpy2.X, time]; ehpy2.Y = [ehpy2.Y, des_p2(2)];
            thpz2.X = [thpz2.X, time]; thpz2.Y = [thpz2.Y, true_p2(3)];
            ehpz2.X = [ehpz2.X, time]; ehpz2.Y = [ehpz2.Y, des_p2(3)];
            
            thpx3.X = [thpx3.X, time]; thpx3.Y = [thpx3.Y, true_p3(1)];
            ehpx3.X = [ehpx3.X, time]; ehpx3.Y = [ehpx3.Y, des_p3(1)];
            thpy3.X = [thpy3.X, time]; thpy3.Y = [thpy3.Y, true_p3(2)];
            ehpy3.X = [ehpy3.X, time]; ehpy3.Y = [ehpy3.Y, des_p3(12)];
            thpz3.X = [thpz3.X, time]; thpz3.Y = [thpz3.Y, true_p3(3)];
            ehpz3.X = [ehpz3.X, time]; ehpz3.Y = [ehpz3.Y, des_p3(3)];
           
            thpx4.X = [thpx4.X, time]; thpx4.Y = [thpx4.Y, true_p4(1)];
            ehpx4.X = [ehpx4.X, time]; ehpx4.Y = [ehpx4.Y, des_p4(1)];
            thpy4.X = [thpy4.X, time]; thpy4.Y = [thpy4.Y, true_p4(2)];
            ehpy4.X = [ehpy4.X, time]; ehpy4.Y = [ehpy4.Y, des_p4(2)];
            thpz4.X = [thpz4.X, time]; thpz4.Y = [thpz4.Y, true_p4(3)];
            ehpz4.X = [ehpz4.X, time]; ehpz4.Y = [ehpz4.Y, des_p4(3)];
            
            thpx5.X = [thpx5.X, time]; thpx5.Y = [thpx5.Y, true_p5(1)];
            ehpx5.X = [ehpx5.X, time]; ehpx5.Y = [ehpx5.Y, des_p5(1)];
            thpy5.X = [thpy5.X, time]; thpy5.Y = [thpy5.Y, true_p5(2)];
            ehpy5.X = [ehpy5.X, time]; ehpy5.Y = [ehpy5.Y, des_p5(2)];
            thpz5.X = [thpz5.X, time]; thpz5.Y = [thpz5.Y, true_p5(3)];
            ehpz5.X = [ehpz5.X, time]; ehpz5.Y = [ehpz5.Y, des_p5(3)];
            
            thpx6.X = [thpx6.X, time]; thpx6.Y = [thpx6.Y, true_p6(1)];
            ehpx6.X = [ehpx6.X, time]; ehpx6.Y = [ehpx6.Y, des_p6(1)];
            thpy6.X = [thpy6.X, time]; thpy6.Y = [thpy6.Y, true_p6(2)];
            ehpy6.X = [ehpy6.X, time]; ehpy6.Y = [ehpy6.Y, des_p6(2)];
            thpz6.X = [thpz6.X, time]; thpz6.Y = [thpz6.Y, true_p6(3)];
            ehpz6.X = [ehpz6.X, time]; ehpz6.Y = [ehpz6.Y, des_p6(3)];
        end