using Microsoft.Kinect;
using Microsoft.Samples.Kinect.ControlsBasics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    public partial class CalibrationWindow : Window
    {
        private KinectSensor m_kinectSensor;
        private Skeleton[] skeletons;

        private List<Point> m_calibPoints = new List<Point>(); //2d calibration points
        private List<SkeletonPoint> m_skeletonCalibPoints = new List<SkeletonPoint>(); //3d skeleton points

        private Matrix3D m_groundPlaneTransform; //step 2 transform
        private Emgu.CV.Matrix<double> m_transform; //step 3 transform


        // OWN VARIABLES
        private int margin = 15;
        private int countPoints = 0;

        private bool calibrated = false;
        private Rectangle dynamicRectangleUnderPerson;

        public CalibrationWindow(KinectSensor sensor)
        {
            InitializeComponent();
            this.m_kinectSensor = sensor;

            dynamicRectangleUnderPerson = new Rectangle
            {
                Fill = Brushes.Red,
                Width = 40,
                Height = 40,
                HorizontalAlignment = HorizontalAlignment.Left,
                VerticalAlignment = VerticalAlignment.Top,
                Visibility = Visibility.Hidden,
                Margin = new Thickness(0, 0, 0, 0)
             
            };
            rectangleBorder.Margin = new Thickness(margin + 5, margin + 5, margin + 5, margin + 5);
            grid.Children.Add(dynamicRectangleUnderPerson);

            

            cornerTopLeft.Margin = new Thickness(margin, margin, 0, 0);
            cornerTopRight.Margin = new Thickness(0, margin, margin, 0);
            cornerBottomRight.Margin = new Thickness(0,0, margin, margin);
            cornerBottomLeft.Margin = new Thickness(margin, 0, 0, margin);

            ShowCorners();
        }

        private void ShowCorners()
        {
            switch (countPoints)
            {
                case 0:
                    cornerTopLeft.Fill = Brushes.Green;
                    break;
                case 1:
                    cornerTopLeft.Fill = Brushes.Red;
                    cornerTopRight.Fill = Brushes.Green;
                    break;
                case 2:
                    cornerTopRight.Fill= Brushes.Red;
                    cornerBottomRight.Fill= Brushes.Green;
                    break;
                case 3:
                    cornerBottomRight.Fill = Brushes.Red;
                    cornerBottomLeft.Fill= Brushes.Green;
                    break;
                default:
                    cornerTopLeft.Visibility = Visibility.Hidden;
                    cornerTopRight.Visibility = Visibility.Hidden;
                    cornerBottomRight.Visibility = Visibility.Hidden;
                    cornerBottomLeft.Visibility = Visibility.Hidden;
                    break;
            }
        }

        private Point GetPositionCornerOnScreen(Ellipse dot)
        {
            var positionTransform = dot.TransformToAncestor(this);
            return positionTransform.Transform(new Point(0, 0));
        }

        public void skeletonStream(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                    if (calibrated)
                    {
                        Point positionPerson = kinectToProjectionPoint(GetSkeletonPoint());
                        dynamicRectangleUnderPerson.Margin = new Thickness(positionPerson.X - margin, positionPerson.Y - margin, 0, 0);
                        dynamicRectangleUnderPerson.Visibility = Visibility.Visible;
                    }

                    foreach (Skeleton skeleton in skeletons)
                    {
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            // RIGHT HAND WAVE
                            if (skeleton.Joints[JointType.HandRight].Position.Y >
                               skeleton.Joints[JointType.ElbowRight].Position.Y)
                            {
                                if (skeleton.Joints[JointType.HandRight].Position.X >
                                    skeleton.Joints[JointType.ElbowRight].Position.X)
                                {
                                    calibrateCorner();
                                    Thread.Sleep(1000);
                                }
                            }
                            // LEFT HAND WAVE
                            if (skeleton.Joints[JointType.HandLeft].Position.Y >
                               skeleton.Joints[JointType.ElbowLeft].Position.Y)
                            {
                                if (skeleton.Joints[JointType.HandLeft].Position.X >
                                    skeleton.Joints[JointType.ElbowLeft].Position.X)
                                {
                                    calibrateCorner();
                                    Thread.Sleep(1000);
                                }
                            }

                        }
                    }

                }
            }
        }
        void calibrateCorner()
        {
            if (countPoints != 4)
            {
                switch (countPoints)
                {
                    case 0:
                        m_calibPoints.Add(GetPositionCornerOnScreen(cornerTopLeft));
                        break;
                    case 1:
                        m_calibPoints.Add(GetPositionCornerOnScreen(cornerTopRight));
                        break;
                    case 2:
                        m_calibPoints.Add(GetPositionCornerOnScreen(cornerBottomRight));
                        break;
                    case 3:
                        m_calibPoints.Add(GetPositionCornerOnScreen(cornerBottomLeft));
                        break;
                    default:
                        break;
                }

                m_skeletonCalibPoints.Add(GetSkeletonPoint());
                countPoints++;
                if (countPoints == 4)
                {
                    rectangleBorder.Visibility = Visibility.Visible;
                    calibrate();
                    calibrated = true;
                    windowMessage.Visibility = Visibility.Hidden;
                }
                ShowCorners();
            }
        }
        
        private SkeletonPoint GetSkeletonPoint()
        {
            if (skeletons != null)
            {
                foreach (Skeleton skeleton in skeletons)
                {
                    if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        return skeleton.Position;
                    }
                }
            }
            return new SkeletonPoint();
        }

        private void calibrate()
        {
            if (m_skeletonCalibPoints.Count == m_calibPoints.Count)
            {
                //seketon 3D positions --> 3d positions in depth camera
                Point3D p0 = conertSkeletonPointToDepthPoint(m_skeletonCalibPoints[0]);
                Point3D p1 = conertSkeletonPointToDepthPoint(m_skeletonCalibPoints[1]);
                Point3D p2 = conertSkeletonPointToDepthPoint(m_skeletonCalibPoints[2]);
                Point3D p3 = conertSkeletonPointToDepthPoint(m_skeletonCalibPoints[3]);

                //3d positions depth camera --> positions on a 2D plane
                Vector3D v1 = p1 - p0;
                v1.Normalize();

                Vector3D v2 = p2 - p0;
                v2.Normalize();

                Vector3D planeNormalVec = Vector3D.CrossProduct(v1, v2);
                planeNormalVec.Normalize();

                Vector3D resultingPlaneNormal = new Vector3D(0, 0, 1);
                m_groundPlaneTransform = Util.make_align_axis_matrix(resultingPlaneNormal, planeNormalVec);

                Point3D p0OnPlane = m_groundPlaneTransform.Transform(p0);
                Point3D p1OnPlane = m_groundPlaneTransform.Transform(p1);
                Point3D p2OnPlane = m_groundPlaneTransform.Transform(p2);
                Point3D p3OnPlane = m_groundPlaneTransform.Transform(p3);

                //2d plane positions --> exact 2d square on screen (using perspective transform)
                System.Drawing.PointF[] src = new System.Drawing.PointF[4];
                src[0] = new System.Drawing.PointF((float)p0OnPlane.X, (float)p0OnPlane.Y);
                src[1] = new System.Drawing.PointF((float)p1OnPlane.X, (float)p1OnPlane.Y);
                src[2] = new System.Drawing.PointF((float)p2OnPlane.X, (float)p2OnPlane.Y);
                src[3] = new System.Drawing.PointF((float)p3OnPlane.X, (float)p3OnPlane.Y);

                System.Drawing.PointF[] dest = new System.Drawing.PointF[4];
                dest[0] = new System.Drawing.PointF((float)m_calibPoints[0].X, (float)m_calibPoints[0].Y);
                dest[1] = new System.Drawing.PointF((float)m_calibPoints[1].X, (float)m_calibPoints[1].Y);
                dest[2] = new System.Drawing.PointF((float)m_calibPoints[2].X, (float)m_calibPoints[2].Y);
                dest[3] = new System.Drawing.PointF((float)m_calibPoints[3].X, (float)m_calibPoints[3].Y);

                Emgu.CV.Mat transform = Emgu.CV.CvInvoke.GetPerspectiveTransform(src, dest);

                m_transform = new Emgu.CV.Matrix<double>(transform.Rows, transform.Cols, transform.NumberOfChannels);
                transform.CopyTo(m_transform);

                //test to see if resulting perspective transform is correct
                //tResultx should be same as points in m_calibPoints
                Point tResult0 = kinectToProjectionPoint(m_skeletonCalibPoints[0]);
                Point tResult1 = kinectToProjectionPoint(m_skeletonCalibPoints[1]);
                Point tResult2 = kinectToProjectionPoint(m_skeletonCalibPoints[2]);
                Point tResult3 = kinectToProjectionPoint(m_skeletonCalibPoints[3]);

                System.Diagnostics.Debug.WriteLine("--TEST WETHER CALIBRATION WORKS CORRECTLY--\n");

                System.Diagnostics.Debug.WriteLine($"Own calib point 0 : x: {m_calibPoints[0].X}, y: {m_calibPoints[0].Y}");
                System.Diagnostics.Debug.WriteLine($"Calculated point 0: x: {tResult0.X}, y: {tResult0.Y}\n");

                System.Diagnostics.Debug.WriteLine($"Own calib point 1 : x: {m_calibPoints[1].X}, y: {m_calibPoints[1].Y}");
                System.Diagnostics.Debug.WriteLine($"Calculated point 1: x: {tResult1.X}, y: {tResult1.Y}\n");

                System.Diagnostics.Debug.WriteLine($"Own calib point 2 : x: {m_calibPoints[2].X}, y: {m_calibPoints[2].Y}");
                System.Diagnostics.Debug.WriteLine($"Calculated point 2: x: {tResult2.X}, y: {tResult2.Y}\n");

                System.Diagnostics.Debug.WriteLine($"Own calib point 3 : x: {m_calibPoints[3].X}, y: {m_calibPoints[3].Y}");
                System.Diagnostics.Debug.WriteLine($"Calculated point 3: x: {tResult3.X}, y: {tResult3.Y}\n");

                System.Diagnostics.Debug.WriteLine("--------------------------------------------");
            }
        }

        private Point3D conertSkeletonPointToDepthPoint(SkeletonPoint skeletonPoint)
        {
            DepthImagePoint imgPt = m_kinectSensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeletonPoint, DepthImageFormat.Resolution640x480Fps30);

            return new Point3D(imgPt.X, imgPt.Y, imgPt.Depth);
        }

        private Point kinectToProjectionPoint(SkeletonPoint point)
        {
            DepthImagePoint depthP = m_kinectSensor.CoordinateMapper.MapSkeletonPointToDepthPoint(point, DepthImageFormat.Resolution640x480Fps30);
            Point3D p = new Point3D(depthP.X, depthP.Y, depthP.Depth);

            Point3D pOnGroundPlane = m_groundPlaneTransform.Transform(p);

            System.Drawing.PointF[] testPoint = new System.Drawing.PointF[1];
            testPoint[0] = new System.Drawing.PointF((float)pOnGroundPlane.X, (float)pOnGroundPlane.Y);

            System.Drawing.PointF[] resultPoint = Emgu.CV.CvInvoke.PerspectiveTransform(testPoint, m_transform);

            return new Point(resultPoint[0].X, resultPoint[0].Y);
        }
    }
}
