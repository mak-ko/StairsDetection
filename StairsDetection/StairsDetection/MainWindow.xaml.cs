using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Diagnostics;
using System.Drawing;
using Microsoft.Kinect;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using System.Globalization;
using System.ComponentModel;
using System.IO;


namespace StairsDetection
{
    public partial class MainWindow : System.Windows.Window
    {
        KinectSensor kinect;
        DepthFrameReader depthFrameReader;
        FrameDescription depthFrameDesc;

        WriteableBitmap imageColor = null;
        ColorFrameReader colorFrameReader;
        FrameDescription colorFrameDesc;
        byte[] colorBuffer;

        WriteableBitmap imageDepth; 
        ushort[] depthBuffer;
        byte[] depthBitmapBuffer;
        Int32Rect depthRect;
        int depthStride;
        System.Windows.Point depthPoint;
        const int R = 20;
        int width = 512;
        int height = 424;
        int cornerCount=600;    //見つかった特徴点の個数

        int lowThreshod = 10;//20        //Canny法のパラメータ
        int highThreshod = 20;//60      
        int houghThreshod = 15; //50 //70    //HoughLine2のパラメータ
        int minLineLength = 10;//50 //80
        int maxLineGap = 5;    //20    //50

        IplImage houghImage;    //hough画像
        IplImage cannyImage;    //canny法によるエッジ画像
        IplImage depthImage;    //kinectからの深度画像
        IplImage flowVectorImage;   //オプティカルフローのベクトル画像
        IplImage prevDepthImage;    //前フレームの深度画像
        IplImage prevPyrImage;      //OpticalFlowPyrLK()のやつ
        IplImage PyrImage;          //          〃
        IplImage colorImage;
        IplImage prevColorImage;
        IplImage prevCannyImage;
        IplImage combineLineImage;
        IplImage stepCandidateImage;
        IplImage stairImage;

        IplImage bodyIndexImage;
        WriteableBitmap imageBodyIndex;
        BodyIndexFrameReader bodyIndexFrameReader;
        FrameDescription bodyIndexFrameDesc;
        byte[] bodyIndexBuffer;
        byte[] bodyIndexColorBuffer;
        System.Windows.Media.Color[] bodyIndexColors;

        CvPoint2D32f[] prevFeaturePoint;       //特徴点
        CvPoint2D32f[] featurePoint;    //1フレーム後，追跡後の特徴点

        CvSeq lines;        //hough変換によって得られる直線たち
            
        CvPoint2D32f[][] horizontalLines = new CvPoint2D32f[2][];       //水平な直線を抽出する
        CvPoint2D32f[][] stairFeaturePoint = new CvPoint2D32f[2][];
        
        CvPoint2D32f[][] stairs = new CvPoint2D32f[2][];

        bool needSave = false;      //color画像を0で保存
        bool stairFlag = false;     //階段を検出したかどうか

        float topLXPoint, topLYPoint = 0;
        float bottomRXPoint, bottomRYPoint = 0;

        int lineCounter = 0;
        int stairCounter = 0;

        public MainWindow()
        {
            InitializeComponent();
        }


        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            try
            {
                kinect = KinectSensor.GetDefault();
                depthFrameDesc = kinect.DepthFrameSource.FrameDescription;
                depthFrameReader = kinect.DepthFrameSource.OpenReader();
                depthFrameReader.FrameArrived += DepthFrameReader_FrameArrived;
                imageDepth = new WriteableBitmap(depthFrameDesc.Width, depthFrameDesc.Height, 
                                                96.0, 96.0, PixelFormats.Gray8, null);
                depthBuffer = new ushort[depthFrameDesc.Width * depthFrameDesc.Height];
                depthBitmapBuffer = new byte[depthFrameDesc.Width * depthFrameDesc.Height];
                depthRect = new Int32Rect(0, 0, depthFrameDesc.Width, depthFrameDesc.Height);
                depthStride = (int)(depthFrameDesc.Width);
                kinect.Open();
                image_depth.Source = imageDepth;
                
                depthPoint = new System.Windows.Point(depthFrameDesc.Width / 2, depthFrameDesc.Height / 2);
                cannyImage = new IplImage(new CvSize(width, height), BitDepth.U8, 1);
                houghImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);
                flowVectorImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);
                prevDepthImage = new IplImage(new CvSize(width, height), BitDepth.U8, 1);
                prevPyrImage = new IplImage(new CvSize(width, height), BitDepth.U8, 1);
                PyrImage = new IplImage(new CvSize(width, height), BitDepth.U8, 1);                
                prevCannyImage = new IplImage(new CvSize(width, height), BitDepth.U8, 1);
                bodyIndexImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);  
                combineLineImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);
                stepCandidateImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);
                stairImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);

                horizontalLines[0] = new CvPoint2D32f[cornerCount];
                horizontalLines[1] = new CvPoint2D32f[cornerCount];
                stairFeaturePoint[0] = new CvPoint2D32f[cornerCount];
                stairFeaturePoint[1] = new CvPoint2D32f[cornerCount];
                stairs[0] = new CvPoint2D32f[cornerCount];
                stairs[1] = new CvPoint2D32f[cornerCount];

                ////RGB画像を使うときはここを！！！
                ////1920x1080だからすごく重くて使いものにならぬ，coordinateMapperを用いてDepth座標系に落とし込む必要有
                colorImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);
                colorFrameDesc = kinect.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
                //prevColorImage = new IplImage(new CvSize(width, height), BitDepth.U8, 3);
                colorFrameReader = kinect.ColorFrameSource.OpenReader();
                colorFrameReader.FrameArrived += ColorFrameReader_FrameArrived;
                imageColor = new WriteableBitmap(colorFrameDesc.Width, colorFrameDesc.Height, 96, 96, PixelFormats.Bgra32, null);
                //image_color.Source = imageColor;
                colorBuffer = new byte[colorFrameDesc.LengthInPixels * colorFrameDesc.BytesPerPixel];

                bodyIndexFrameDesc = kinect.BodyIndexFrameSource.FrameDescription;
                bodyIndexFrameReader = kinect.BodyIndexFrameSource.OpenReader();
                bodyIndexFrameReader.FrameArrived += BodyIndexFrameReader_FrameArrived;
                bodyIndexBuffer = new byte[bodyIndexFrameDesc.LengthInPixels];
                imageBodyIndex = new WriteableBitmap(bodyIndexFrameDesc.Width, bodyIndexFrameDesc.Height, 96, 96, PixelFormats.Bgra32, null);
                
                bodyIndexColorBuffer = new byte[bodyIndexFrameDesc.LengthInPixels * 4];
                bodyIndexColors = new System.Windows.Media.Color[]
                {
                    Colors.Red, Colors.Blue, Colors.Green, Colors.Yellow, Colors.Pink, Colors.Purple,
                };
                //image_bodyIndex.Source = imageBodyIndex;

                image_canny.Source = cannyImage.ToWriteableBitmap();
                image_hough.Source = houghImage.ToWriteableBitmap();
                //image_flowVector.Source = flowVectorImage.ToWriteableBitmap();


                ths.Text = "lowThreshod=" + lowThreshod +
                            "\thighThreshod=" + highThreshod +
                            "\thoughThreshod=" + houghThreshod +
                            "\tminLineLength=" + minLineLength +
                            "\tmaxLineGap=" + maxLineGap;
            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.Message);
                Close();
            }
        }


        private void BodyIndexFrameReader_FrameArrived(object sender, BodyIndexFrameArrivedEventArgs e)
        {
            using(var bodyIndexFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyIndexFrame == null)
                {
                    return;
                }

                bodyIndexFrame.CopyFrameDataToArray(bodyIndexBuffer);
                
            }

            for(int i=0; i<bodyIndexBuffer.Length; i++)
            {
                var index = bodyIndexBuffer[i];
                var colorIndex = i * 4;
                //Debug.WriteLine();
                if(index != 255)
                {
                    var color = bodyIndexColors[index];
                    bodyIndexColorBuffer[colorIndex + 0] = color.B;
                    bodyIndexColorBuffer[colorIndex + 1] = color.G;
                    bodyIndexColorBuffer[colorIndex + 2] = color.R;
                    bodyIndexColorBuffer[colorIndex + 3] = 255;

                }
                else
                {
                    bodyIndexColorBuffer[colorIndex + 0] = 0;
                    bodyIndexColorBuffer[colorIndex + 1] = 0;
                    bodyIndexColorBuffer[colorIndex + 2] = 0;
                    bodyIndexColorBuffer[colorIndex + 3] = 255;
                }
            }




            imageBodyIndex.WritePixels(new Int32Rect(0, 0, bodyIndexFrameDesc.Width, bodyIndexFrameDesc.Height), bodyIndexColorBuffer, bodyIndexFrameDesc.Width * 4, 0);
            
        }


        //color frameが来た時に実行される//カラー画像データ

        private void ColorFrameReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            if (!needSave) return;//needSave:保存時に立つフラグ

            using (var colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame == null)
                {
                    return;
                }
                    colorFrame.CopyConvertedFrameDataToArray(colorBuffer, ColorImageFormat.Bgra);

                    imageColor.WritePixels(
                        new Int32Rect(0, 0, colorFrameDesc.Width, colorFrameDesc.Height),
                        colorBuffer, colorFrameDesc.Width * (int)colorFrameDesc.BytesPerPixel, 0);


                IplImage stairsSave = imageColor.ToIplImage();
                stairsSave.Rectangle(new CvPoint2D32f(topLXPoint - 10, topLYPoint - 10), 
                    new CvPoint2D32f(bottomRXPoint + 10, bottomRYPoint + 10),CvColor.Orange, 1);//IplImageのcoloImageに四角を描く
                WriteableBitmap SaveImage = stairsSave.ToWriteableBitmap();//IplImage to WriteblaBitmap

                if (imageColor != null)
                    {
                        if (stairFlag)  //階段があるなら撮る
                        {
                            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);//日時取得
                            using (FileStream stream = new FileStream("KinectScreenshot-Color-" + time + ".bmp", FileMode.Create, FileAccess.Write))
                            {
                                BmpBitmapEncoder encorder = new BmpBitmapEncoder();
                            //encorder.Frames.Add(BitmapFrame.Create(imageColor));//writebleBitmapの四角なし保存
                            encorder.Frames.Add(BitmapFrame.Create(SaveImage));//四角ありの画像保存
                            encorder.Save(stream);
                                MessageBox.Show("Founded Stairs.");
                                stairFlag = false;
                            }
                        }
                        else
                        {
                        MessageBox.Show("NotFound Stairs.");
                    }
                    }
                    else
                    {
                        MessageBox.Show("NULL");
                    }

                    needSave = false;
            }
        }

        
        //depth frameが来た時に実行される//深度画像データ
        
        private void UpdateDepthFrame(DepthFrameArrivedEventArgs e)
        {
            using (var depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame == null) return;

                depthFrame.CopyFrameDataToArray(depthBuffer);
            }
            //UpdateDepthValue();
            for (int i = 0; i < depthBuffer.Length; i++)
            {
                //depthBitmapBuffer[i] = (byte)(depthBuffer[i]);      //深度に波がある
                depthBitmapBuffer[i] = (byte)(depthBuffer[i] / (8000 / 255)); //なめらか！！！なんで？，参考：Microsoftのサンプルプログラム
                 
            }
            imageDepth.WritePixels(depthRect, depthBitmapBuffer, depthFrameDesc.Width, 0);
        }


        private void UpdateDepthValue()
        { 
            CanvasPoint.Children.Clear();
            var ellipse = new Ellipse()
            {
                Width = R,
                Height = R,
                StrokeThickness = R / 4,
                Stroke = System.Windows.Media.Brushes.Red,
            };

            Canvas.SetLeft(ellipse, depthPoint.X- R/2);
            Canvas.SetTop(ellipse, depthPoint.Y-R/2);
            CanvasPoint.Children.Add(ellipse);
          
            int depthIndex = (int)((depthPoint.Y * width) + depthPoint.X);
            if (depthIndex >= width*height)
            {
                depthIndex = 0;
            }
            
            var text = new TextBlock()
            {
                Text = string.Format("{0}mm", depthBuffer[depthIndex]),
                FontSize = 20,
                Foreground = System.Windows.Media.Brushes.Green,
            };

            Canvas.SetLeft(text, depthPoint.X);
            Canvas.SetTop(text, depthPoint.Y - R/2);
            CanvasPoint.Children.Add(text);
        }


        /*  階段の推定を行う関数
        */
        private void CalcEdge()
        {
            horizontalLines.Initialize();
            Cv.Smooth(depthImage, depthImage, SmoothType.Median, 3, 3);     //メディアンフィルタ，日光の紫外線によるノイズを低減させるため
            Cv.Canny(depthImage, cannyImage, lowThreshod, highThreshod, ApertureSize.Size3);          //Canny法でエッジ検出  
            lines = Cv.HoughLines2(cannyImage, new CvMemStorage(),HoughLinesMethod.Probabilistic, 
                                         1, Math.PI/180, houghThreshod, minLineLength, maxLineGap);

            lineCounter = 0;
            for(int i=0; i<lines.Total; i++)    //全ての見つかった線について
            {     
                CvLineSegmentPoint elem = lines.GetSeqElem<CvLineSegmentPoint>(i).Value;
                CvPoint start = elem.P1;    //直線の始点
                CvPoint end = elem.P2;      //直線の終点
                houghImage.Line(start, end, CvColor.White, 1);    //すべての直線を描画，確認用

                //----とりあえず，水平な直線だけを取り出してみる-------------------------------------------------------------------------------
                if (Math.Abs((double)start.Y - (double)end.Y) / Math.Abs((double)start.X - (double)end.X) < 0.15)    //斜めや縦に伸びる線は除外，勾配を計算している
                //if(start.Y - end.Y <= 20)
                                                                       //↑ゼロ除算の可能性あるけど，(double)でキャストすれば大丈夫だった，謎
                {
                    horizontalLines[0][lineCounter] = start;
                    horizontalLines[1][lineCounter++] = end;
                    //houghImage.Line(start, end, CvColor.White, 1);    //水平な直線だけを描画，確認用
                }
            }

            int[] ids = new int[lineCounter];
            for (int i = 0; i < lineCounter; i++) ids[i] = -1;

            //----全線分で，その線分の中央座標の上下の深度差を調べる．---------------------------------------------------------↓
            CvPoint2D32f[] mid = new CvPoint2D32f[lineCounter];
            for (int i = 0; i < lineCounter; i++)
            {
                //各線分の中央座標
                mid[i] = new CvPoint2D32f((horizontalLines[1][i].X + horizontalLines[0][i].X) / 2,
                                                       (horizontalLines[1][i].Y + horizontalLines[0][i].Y) / 2);

                //houghImage.Circle(mid[i], 3, CvColor.AliceBlue, 4);
                int upper = ((int)mid[i].Y - 5) * width + (int)mid[i].X;    //中央座標の5pixel上方
                int lower = ((int)mid[i].Y + 5) * width + (int)mid[i].X;    //中央座標の5pixel下方
                float diff = 0;
                if (upper < 0 || lower >= width * height)   //IndexOutOfBoundsを起こすことがあるので，ごまかす（ダメ）
                {
                    diff = 0;   
                }
                else
                {
                    diff = depthBuffer[upper] - depthBuffer[lower]; //深度差を計算するの
                }

                if (diff >= 100 && diff <= 500)
                {
                    stepCandidateImage.Line(horizontalLines[0][i], horizontalLines[1][i], CvColor.Red, 1);  //深度差がいい感じなら赤
                }
                else
                {
                    ids[i] = -2;    //この線分を無視するために，IDを-2にする．
                    stepCandidateImage.Line(horizontalLines[0][i], horizontalLines[1][i], CvColor.White, 1);    //深度差がやばみなら白
                }
            }
            //------------------------------------------------------------------------------------------------------------------↑

            CvPoint2D32f[][] combinedLine = new CvPoint2D32f[2][];
            combinedLine[0] = new CvPoint2D32f[100];
            combinedLine[1] = new CvPoint2D32f[100];
            combinedLine[0][0] = horizontalLines[0][0];
            combinedLine[1][0] = horizontalLines[1][0];
            
            //----同一線上にある線分に同じIDを付与
            int c = 0;
            for(int i=0; i < lineCounter; i++)     
            {
                if (ids[i] != -1 || ids[i] == -2) continue;
                ids[i] = c++;
                for (int j = 0; j < lineCounter; j++)
                {
                    if ( i==j || ids[j] != -1) continue;
                    if(Math.Abs(horizontalLines[0][i].Y - horizontalLines[0][j].Y) <= 5)
                    {
                        ids[j] = ids[i];
                    }    
                }
            }

            //確認用，同一線上ID
            for(int i=0; i< lineCounter; i++)
            {
                if (ids[i] != -2)
                {
                    //houghImage.Line(horizontalLines[0][i], horizontalLines[1][i], CvColor.Red);
                    //houghImage.PutText("" + ids[i], horizontalLines[0][i], new CvFont(FontFace.HersheyScriptSimplex, 0.5f, 0.5f), CvColor.Green);
                }
            }

            float[] depth = new float[c];
            for(int i = 0; i < c; i++)
            {
                float[] temp = new float[10];
            }
       
            //----同じIDの線分どうしを結合する----------------------------------------------------------------------------------------↓
            for (int i=0; i<c; i++)
            {
                CvPoint2D32f[][] temp = new CvPoint2D32f[2][];
                temp[0] = new CvPoint2D32f[50];
                temp[1] = new CvPoint2D32f[50];
                float[] d = new float[50];
                int countUP = 0;
                for (int j=0; j<lineCounter; j++)
                {
                    if (ids[j] == i)
                    {
                        temp[0][countUP] = horizontalLines[0][j];
                        temp[1][countUP] = horizontalLines[1][j];
                        CvPoint2D32f centre = new CvPoint2D32f((horizontalLines[1][j].X + horizontalLines[0][j].X) / 2,
                                                       (horizontalLines[1][j].Y + horizontalLines[0][j].Y) / 2);
                        int index = (int)(centre.Y + 5) * width + (int)centre.X;
                        if (index >= width * height)
                        {
                            d[countUP++] = -1;
                        }
                        else
                        {
                            d[countUP++] = depthBuffer[index];
                        }
                    }         
                }

                float sum = 0;
                for(int j=0; j < countUP; j++)
                {
                    sum += d[j];
                }
                depth[i] = sum / countUP;

                double minX = temp[0][0].X;
                double minY = temp[0][0].Y;
                double maxX = temp[1][0].X;
                double maxY = temp[1][0].Y;

                //始点と終点を決定する
                for (int j = 0; j < countUP; j++)
                {
                    if (minX >= temp[0][j].X)
                    {
                        minX = temp[0][j].X;
                        minY = temp[0][j].Y;
                    }
                    if (maxX <= temp[1][j].X)
                    {
                        maxX = temp[1][j].X;
                        maxY = temp[1][j].Y;
                    }
                    //Debug.WriteLine("" + d[j]);
                }

                //結合したLineとして登録
                combinedLine[0][i] = new CvPoint2D32f(minX, minY);
                combinedLine[1][i] = new CvPoint2D32f(maxX, maxY);
                combineLineImage.Line(new CvPoint((int)minX, (int)minY), new CvPoint((int)maxX, (int)maxY), CvColor.Red, 1);
                //Debug.WriteLine("---");
            }
            //-------------------------------------------------------------------------------------------------------------------------↑

            CvPoint2D32f[][] stairCanditates = new CvPoint2D32f[2][];
            stairCanditates[0] = new CvPoint2D32f[c];
            stairCanditates[1] = new CvPoint2D32f[c];
            int tmpStairCounter = 0;

            //----depthとcombinedLineをソートする
            for(int i = 0; i < c; i++)
            {
                for(int j = c; j > i; j--)
                {
                    if(combinedLine[0][j].Y > combinedLine[0][i].Y)
                    {
                        CvPoint2D32f start = combinedLine[0][j];
                        CvPoint2D32f end = combinedLine[1][j];
                        combinedLine[0][j] = combinedLine[0][i];
                        combinedLine[1][j] = combinedLine[1][i];
                        combinedLine[0][i] = start;
                        combinedLine[1][i] = end;

                        float s = depth[j];
                        depth[j] = depth[i];
                        depth[i] = s;
                    }
                }
            }

            //確認用
            for (int i = 0; i < c; i++)
            {
                combineLineImage.Line(combinedLine[0][i], combinedLine[1][i], CvColor.Red);
                //houghImage.PutText("" + depth[i], combinedLine[1][i], new CvFont(FontFace.HersheyScriptSimplex, 0.5f, 0.5f), CvColor.Red);
                //combineLineImage.PutText("" + depth[i], combinedLine[1][i], new CvFont(FontFace.HersheyScriptSimplex, 0.5f, 0.5f), CvColor.Red);
            }
            
            //----直線群から階段の候補を取り出す------------------------------
            for (int i = 0; i < c; i++)
            {
                CvPoint2D32f[][] tmp = new CvPoint2D32f[2][];
                tmp[0] = new CvPoint2D32f[100];
                tmp[1] = new CvPoint2D32f[100];
                int lineNum = 0;
                CvPoint2D32f center = new CvPoint2D32f((combinedLine[1][i].X + combinedLine[0][i].X) / 2,
                                                       (combinedLine[1][i].Y + combinedLine[0][i].Y) / 2);

                //なんかここら辺怪しい．
                for (int j = 0; j < c; j++)
                {
                    if (center.X >= combinedLine[0][j].X && center.X <= combinedLine[1][j].X && depth[i] <= depth[j])      //線分の間にあるとき
                    {
                        
                        tmp[0][lineNum] = combinedLine[0][j];
                        tmp[1][lineNum++] = combinedLine[1][j];    //いったん保持
                    }
                }
                if (lineNum >= tmpStairCounter)     //段数が最も多いものを階段候補とする
                {
                    tmpStairCounter = lineNum;
                    stairCanditates = tmp;
                }
            }
            
            //ただの変数名合わせ
            stairCounter = 0;
            for (int i = 0; i < tmpStairCounter; i++)
            {
                if (true)
                {
                    stairs[0][stairCounter] = stairCanditates[0][i];
                    stairs[1][stairCounter++] = stairCanditates[1][i];
                }
            }

            //階段領域を四角で囲みたかった
            stairFlag = false;
            if (stairCounter >= 2)
            {
                stairFlag = true;
                float maxX = 0;
                float maxY = 0;
                float minX = width;
                float minY = height;
                for(int i=0; i<stairCounter; i++)
                {
                    if(minX >= stairs[0][i].X || minX >= stairs[1][i].X)
                    {
                        minX = Math.Min(stairs[0][i].X, stairs[1][i].X);   
                    }
                    if (minY >= stairs[0][i].Y || minY >= stairs[1][i].Y)
                    {
                        minY = Math.Min(stairs[0][i].Y, stairs[1][i].Y);
                    }
                    if (maxX <= stairs[0][i].X || maxX <= stairs[1][i].X)
                    {
                        maxX = Math.Max(stairs[0][i].X, stairs[1][i].X);
                    }
                    if (maxY <= stairs[0][i].Y || maxY <= stairs[1][i].Y)
                    {
                        maxY = Math.Max(stairs[0][i].Y, stairs[1][i].Y);
                    }
                }
                stairImage.Rectangle(new CvPoint2D32f(minX-10, minY-10), new CvPoint2D32f(maxX+10, maxY+10),
                                     CvColor.Orange, 1);
                topLXPoint = minX;
                topLYPoint = minY;
                bottomRXPoint = maxX;
                bottomRYPoint = maxY;
            }
            
            for (int i = 0; i < stairCounter; i++)
            {
                stairImage.Line(stairs[0][i], stairs[1][i], CvColor.Red, 1);    //階段を描画
                stairImage.PutText("" + (i + 1), stairs[0][i], new CvFont(FontFace.HersheyScriptSimplex, 0.5f, 0.5f), CvColor.Red); //段数を付与
                stairFlag = true;
            }

            ////距離を表示する
            //int index2 = (int)((combinedLine[0][0].Y + 5) * width + combinedLine[0][0].X);
            //double dist = depthBuffer[index2];
            //stairImage.PutText("" + dist + "mm", new CvPoint(150, 380), new CvFont(FontFace.HersheyScriptSimplex, 2, 2), CvColor.Red);
            //distance.Text = dist + "mm";
        }


        /*  特徴点抽出のアルゴリズムを実装した関数
            cornerCountはここで毎回書き換わる
            1フレーム前の深度画像について特徴点を探す
            prevFeaturePointに特徴点を格納する
        */
        private void FindFeturePoint()
        {
            IplImage tempImage = new IplImage(depthImage.GetSize(), BitDepth.U8, 1);    //this parameter is ignored.
            IplImage eigImage = new IplImage(depthImage.GetSize(), BitDepth.U8, 1);     //this parameter is ignored.

            cornerCount = 300;              //下記のGoodFeaturesToTrack()を通るたびに見つかった特徴点の個数に書き換わるので注意，だから毎回初期化する
            Cv.GoodFeaturesToTrack(prevCannyImage, eigImage, tempImage, 
                                   out prevFeaturePoint, ref cornerCount,
                                   0.1, 10);  
            //Cv.FindCornerSubPix(prevDepthImage,  prevFeaturePoint, cornerCount, new CvSize(5,5), new CvSize(-1,-1), new CvTermCriteria(10, 0.1));     //？

        //----直線付近の特徴点を抽出する-----------------------↓         //見つからなかった時の処理はどうなるの？
            //始点について     
            for (int j = 0; j < stairCounter; j++)
            {
                bool find = false;
                CvPoint2D32f tmp = prevFeaturePoint[0];
                int nearestPointindex = 0;
                for(int i=0; i<cornerCount; i++)
                {
                    if(Math.Abs(prevFeaturePoint[i].X- stairs[0][j].X) <= 10 && Math.Abs(prevFeaturePoint[i].Y - stairs[0][j].Y) <=10)         //始点
                    {   
                        //階段の始点に一番近い特徴点を，階段の始点の特徴点とする
                        if (Math.Pow(tmp.X - stairs[0][j].X, 2) + Math.Pow(tmp.Y - stairs[0][j].Y,2) >= 
                            Math.Pow(prevFeaturePoint[i].X - stairs[0][j].X,2) + Math.Pow(prevFeaturePoint[i].Y - stairs[0][j].Y,2))
                        {
                            tmp = prevFeaturePoint[i];
                            nearestPointindex = i;
                            find = true;
                        }
                    }
                }
                if (find == true)
                {
                    stairFeaturePoint[0][j] = prevFeaturePoint[nearestPointindex];
                }
                else if(find == false)  //見つからなかったときは，前回見つけたやつの中から一番近いやつにすればいいと思う
                {
                    
                }
            }

            //終点について
            for (int j = 0; j < stairCounter; j++)
            {
                bool find = false;
                CvPoint2D32f tmp = prevFeaturePoint[0];
                int nearestPointindex = 0;

                for (int i = 0; i < cornerCount; i++)
                {
                    if (Math.Abs(prevFeaturePoint[i].X - stairs[1][j].X) <= 10 && Math.Abs(prevFeaturePoint[i].Y - stairs[1][j].Y) <= 10)         //始点
                    {
                        if (Math.Pow(tmp.X - stairs[1][j].X, 2) + Math.Pow(tmp.Y - stairs[1][j].Y, 2) >= 
                            Math.Pow(prevFeaturePoint[i].X - stairs[1][j].X, 2) + Math.Pow(prevFeaturePoint[i].Y - stairs[1][j].Y, 2))
                        {
                            tmp = prevFeaturePoint[i];
                            nearestPointindex = i;
                            find = true;
                        }
                    }
                }
                if (find == true)
                {
                    stairFeaturePoint[1][j] = prevFeaturePoint[nearestPointindex];
                }
            }
        //-----------------------------------------------------↑

            //確認用，すべての特徴点を描画，白い点
            for (int i=0; i< cornerCount; i++)   
            {
                houghImage.Circle(prevFeaturePoint[i], 1, CvColor.White, 1);
            }

            //階段の両端の特徴点
            for (int i = 0; i < stairCounter; i++)
            {
                combineLineImage.Circle(stairFeaturePoint[0][i], 3, CvColor.Green, 2);
                combineLineImage.Circle(stairFeaturePoint[1][i], 3, CvColor.Yellow, 2);
            }
        }
        

        /*  オプティカルフローで特徴点を追跡する
            featurePointに追跡後の特徴点を格納する
        */
        private void CalcOpticalFlow()
        {
            sbyte[] features_status = new sbyte[cornerCount];
            float[] all_features_errors = new float[cornerCount];
            
            CvPoint2D32f[] fpStart;
            CvPoint2D32f[] fpEnd;

            ////全ての特徴点について
            //Cv.CalcOpticalFlowPyrLK(prevDepthImage, depthImage,
            //                        prevPyrImage, PyrImage,
            //                        prevFeaturePoint, out featurePoint,
            //                        new CvSize(10, 10), 5,
            //                        out features_status, out all_features_errors,
            //                        new CvTermCriteria(20, 0.3), 0);

            //階段の始点について
            Cv.CalcOpticalFlowPyrLK(prevDepthImage, depthImage,
                                    prevPyrImage, PyrImage,
                                    stairFeaturePoint[0], out fpStart,
                                    new CvSize(10, 10), 5,
                                    out features_status, out all_features_errors,
                                    new CvTermCriteria(20, 0.3), 0);
            //階段の終点について
            Cv.CalcOpticalFlowPyrLK(prevDepthImage, depthImage,
                                  prevPyrImage, PyrImage,
                                  stairFeaturePoint[1], out fpEnd,
                                  new CvSize(10, 10), 5,
                                  out features_status, out all_features_errors,
                                  new CvTermCriteria(20, 0.3), 0);

            //ベクトルを描画，階段の始点と終点のみ
            for (int i = 0; i < stairCounter; i++)
            {
                CvPoint prevP = new CvPoint((int)stairFeaturePoint[0][i].X, (int)stairFeaturePoint[0][i].Y);
                CvPoint currP = new CvPoint((int)fpStart[i].X, (int)fpStart[i].Y);
                flowVectorImage.Line(prevP, currP, CvColor.Green, 3);

                prevP = new CvPoint((int)stairFeaturePoint[1][i].X, (int)stairFeaturePoint[1][i].Y);
                currP = new CvPoint((int)fpEnd[i].X, (int)fpEnd[i].Y);
                flowVectorImage.Line(prevP, currP, CvColor.Yellow, 3);
            }

            //ベクトルを描画，すべて，確認用
            //for(int i=0; i< cornerCount; i++)
            //{
            //    CvPoint prevP = new CvPoint((int)prevFeaturePoint[i].X, (int)prevFeaturePoint[i].Y);
            //    CvPoint currP = new CvPoint((int)featurePoint[i].X, (int)featurePoint[i].Y);
            //    flowVectorImage.Line(prevP, currP, CvColor.Green, 2);
            //}
        }


        /*  深度画像フレームが来る毎に実行されるやつ
        */
        private void DepthFrameReader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
        //----Kinectを用いて深度画像を取得する，その他画像の初期化-----------------------↓
            UpdateDepthFrame(e);
            depthImage = imageDepth.ToIplImage();
            colorImage = imageColor.ToIplImage();
            cannyImage.Zero();
            houghImage.Zero();
            flowVectorImage.Zero();
            combineLineImage.Zero();
            stepCandidateImage.Zero();
            stairImage.Zero();

        //----実装したアルゴリズム諸々を実行----------------------------------------------↓ 
            CalcEdge();                     //階段検出
            //FindFeturePoint();             //特徴点抽出
            //CalcOpticalFlow();              //オプティカルフローの計算

            //----前フレームの画像を保持する----
            PyrImage.Copy(prevDepthImage);
            depthImage.Copy(prevDepthImage);
            cannyImage.Copy(prevCannyImage);

        //----表示する画像のソースを指定する-----------------------------
            image_canny.Source = cannyImage.ToWriteableBitmap();        
            image_hough.Source = houghImage.ToWriteableBitmap();
            //image_flowVector.Source = flowVectorImage.ToWriteableBitmap();
            //image_bodyIndex.Source = imageBodyIndex;
            image_combinedLine.Source = combineLineImage.ToWriteableBitmap();
            image_stepCandidate.Source = stepCandidateImage.ToWriteableBitmap();
            image_stairs.Source = stairImage.ToWriteableBitmap();
            //Debug.WriteLine("kamome");            
        }


        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (depthFrameReader != null)
            {
                depthFrameReader.Dispose();
                depthFrameReader = null;
            }
            if(colorFrameReader != null)
            {
                colorFrameReader.Dispose();
                colorFrameReader = null;
            }
            if (kinect != null)
            {
                kinect.Close();
                kinect = null;
            }
        }


        /*  何らかのkeyが押された時の処理
            閾値とかを変える
        */
        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Q) lowThreshod++;
            if (e.Key == Key.A) lowThreshod--;
            if (e.Key == Key.W) highThreshod++;
            if (e.Key == Key.S) highThreshod--;
            if (e.Key == Key.E) houghThreshod++;
            if (e.Key == Key.D) houghThreshod--;
            if (e.Key == Key.R) minLineLength++;
            if (e.Key == Key.F) minLineLength--;
            if (e.Key == Key.T) maxLineGap++;
            if (e.Key == Key.G) maxLineGap--;
            if (e.Key == Key.Escape) Close();
            if (e.Key == Key.D0)
            {
                needSave = true;
            }
            if(e.Key == Key.Enter)
            {

            }


            ths.Text = "lowThreshod=" + lowThreshod +
                            "\thighThreshod=" + highThreshod +
                            "\thoughThreshod=" + houghThreshod +
                            "\tminLineLength=" + minLineLength +
                            "\tmaxLineGap=" + maxLineGap;

            Debug.WriteLine("lowThreshod=" + lowThreshod +
                            "\thighThreshod=" + highThreshod +
                            "\thoughThreshod=" + houghThreshod +
                            "\tminLineLength="+ minLineLength +
                            "\tmaxLineGap="+ maxLineGap);
        }


        private void Window_MouseMove(object sender, MouseEventArgs e)
        {
            depthPoint = e.GetPosition(this);
            depthPoint = new System.Windows.Point(depthPoint.X-212-8, depthPoint.Y);
        }

            /*if(imageColor != null)
        {
            BitmapEncoder encorder = new PngBitmapEncoder();
            encorder.Frames.Add(BitmapFrame.Create(this.imageColor));
            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);//日時取得
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.Personal);
            string path = System.IO.Path.Combine(myPhotos, "KinectScreenshot-Color-" + time + ".png");
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encorder.Save(fs);
                }

                //this.StatusText = string.Format(Properties.Resources.SavedScreenshotStatusTextFormat, path);
            }
            catch (IOException)
            {
                //this.StatusText = string.Format(Properties.Resources.FailedScreenshotStatusTextFormat, path);
            }

        }*/
    }
}
