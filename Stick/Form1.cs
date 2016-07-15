using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Stick
{
   

    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
          
        }

        

        private void pictureBox1_Click(object sender, EventArgs e)
        {
           
           
        }

        public delegate void aufruf(double[][] Angles, int Length, int Time, int round); 

        // duration = duration of simulation in seconds
        // time = how many milliseconds should be one time unit for the GA
        public void TestGA(int duration, int time)
        {
            GeneticAlgorithm GA = new GeneticAlgorithm();
            Adam Dummy = new Adam(1, 1, 1, 1);
            aufruf MyAufruf = new aufruf(Display);
            GA.Go(duration, time, Dummy.JointDictionary.Count, pictureBox1.Width, pictureBox1.Height, 10, 10, MyAufruf);
        }

        public void Display(double[][] Angles, int Length, int Time, int round)
        {
            Adam Adam1;
            Adam1 = new Adam(pictureBox1.Width, pictureBox1.Height, 10, 10);
            label2.Text = round.ToString();
            String Test = Adam1.SimulateAngles(Angles, Time, true, pictureBox1).ToString();
            label3.Text = Test;
        }

        public void TestRunUdo()
        {
            Adam Adam1;
            Adam1 = new Adam(pictureBox1.Width, pictureBox1.Height, 10, 10);
            /*int p1 = Adam1.AddPointMass(0.1, 1.7, 20);//Head/Shoulders
        int p2 = Adam1.AddPointMass(0, 0.9, 20);//Hip
        Adam1.AddSpring(p1, p2, 0.8);
        int p3 = Adam1.AddPointMass(-0.1, 0.5, 8);//Knee1
        Adam1.AddSpring(p2, p3, 0.4);
        Adam1.AddJoint(p1, p3, p2, 195);//HipKnee1*/
            List<double[]> Angles = new List<double[]>();
            for (int i = 0; i < 10000; i++)
            {
                Angles.Add(new double[] { 245, 180, 115, 180, 45, 180, -45, 180 });
            }
            for (int i = 0; i < 10000; i++)
            {
                Angles.Add(new double[] { 245 - 245 * i / 10000, 180 + 225 * i / 10000, 115, 180, 45, 180, -45, 180 });
            }
            for (int i = 0; i < 10000; i++)
            {
                Angles.Add(new double[] { 0, 45 - 225 * i / 10000, 115, 180, 45, 180, -45, 180 });
            }
            for (int i = 0; i < 10000; i++)
            {
                Angles.Add(new double[] { 0 + 45 * i / 10000, 180, 115 - 115 * i / 10000, 180 + 225 * i / 10000, 45, 180, -45, 180 });
            }
            for (int i = 0; i < 60000; i++)
            {
                Angles.Add(new double[] { 45, 180, 0, 45, 45, 180, -45, 180 });
            }



            Adam1.Initialize();
            double TimeStep = 0.0001;
            double MaxTime = 10;
            double Steps = MaxTime / TimeStep;
            for (int t0 = 0; t0 < Steps / 100; t0++)
            {
                pictureBox1.Image = Adam1.Draw();
                for (int i = 0; i < 10; i++)
                {
                    for (int j = 0; j < 10; j++)
                    {
                        Adam1.SetAngles(Angles[t0 * 100 + 10 * i + 1 * j]);
                        Adam1.Update(TimeStep);
                    }

                    //Sleep TimeStep
                }
                Thread.Sleep(10);
                Application.DoEvents();
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            TestRunUdo();
        }

        private void button2_Click(object sender, EventArgs e)
        {
            TestGA(2, 100);
        }
    }

  
    public class Adam
    {
        public List<PointMass> PointMassDictionary = new List<PointMass>();
        public List<Spring> SpringDictionary = new List<Spring>();
        public List<Joint> JointDictionary = new List<Joint>();
        public int WorldWidth;
        public int WorldHeight;
        public double WorldWidthReal;
        public double WorldHeightReal;
        public Adam(int _WorldWidth, int _WorldHeight, double _WorldWidthReal, double _WorldHeightReal)
        {
            WorldHeight = _WorldHeight;
            WorldWidth = _WorldWidth;
            WorldHeightReal = _WorldHeightReal;
            WorldWidthReal = _WorldWidthReal;

            int Offset = 2;

            int p1 = AddPointMass(0 + Offset, 1.7, 20);//Head/Shoulders
            int p2 = AddPointMass(0 + Offset, 0.9, 20);//Hip
            AddSpring(p1, p2, 0.8);

            int p3 = AddPointMass(-0.1 + Offset, 0.5, 8);//Knee1
            AddSpring(p2, p3, 0.4);
            int j1 = AddJoint(p1, p3, p2, 245);//HipKnee1

            int p4 = AddPointMass(-0.2 + Offset, 0.1, 4);//Foot1
            AddSpring(p3, p4, 0.5);
            int j2 = AddJoint(p2, p4, p3, 180);//Knee1Joint

            int p5 = AddPointMass(0.1 + Offset, 0.5, 8);//Knee2
            AddSpring(p2, p5, 0.4);
            int j3 = AddJoint(p1, p5, p2, 115);//HipKnee2

            int p6 = AddPointMass(0.2 + Offset, 0.1, 4);//Foot2
            AddSpring(p5, p6, 0.5);
            int j4 = AddJoint(p2, p6, p5, 180);//Knee2Joint

            int p7 = AddPointMass(0.35 + Offset, 1.35, 5);//Knee1
            AddSpring(p1, p7, 0.5);
            int j5 = AddJoint(p2, p7, p1, 45);//HipKnee1

            int p8 = AddPointMass(0.7 + Offset, 1, 3);//Foot1
            AddSpring(p7, p8, 0.5);
            int j6 = AddJoint(p1, p8, p7, 180);//Knee1Joint

            int p9 = AddPointMass(-0.35 + Offset, 1.35, 5);//Knee2
            AddSpring(p1, p9, 0.5);
            int j7 = AddJoint(p2, p1, p9, -45);//HipKnee2

            int p10 = AddPointMass(-0.7 + Offset, 1, 3);//Foot2
            AddSpring(p9, p10, 0.5);
            int j8 = AddJoint(p1, p10, p9, 180);//Knee2Joint
        }

        public double SimulateAngles(double[][] _Angles, int Time, bool _draw, PictureBox pictureBox1)
        {
            List<double[]> Angles = new List<double[]>();

            for (int i = 0; i < _Angles.Length; i++)
            {
                for (int j = 0; j < 10 * Time ; j++)
                    Angles.Add(_Angles[i]);
            }


            double TimeStep = 0.0001;
            //double MaxTime = 10;
            double Steps = Angles.Count;
            for (int t0 = 0; t0 < Steps / 100; t0++)
            {
                if (_draw)
                    pictureBox1.Image = Draw();
                for (int i = 0; i < 10; i++)
                {
                    for (int j = 0; j < 10; j++)
                    {
                        SetAngles(Angles[t0 * 100 + 10 * i + 1 * j]);
                        Update(TimeStep);
                    }

                    //Sleep TimeStep
                }
                if (_draw) { 
                Thread.Sleep(10);
                Application.DoEvents();
                }
            }

            if (PointMassDictionary.ElementAt(0).Position.Item1 > 0)
                return PointMassDictionary.ElementAt(0).Position.Item1;
            else
                return 0;
        }

        public int AddPointMass(double PosX, double PosY, double Mass)
        {
            PointMassDictionary.Add(new PointMass(new Tuple<double, double>(PosX, PosY), Mass));
            return (PointMassDictionary.Count()-1);
        }

        public int AddSpring(int _Mass1Index, int _Mass2Index, double _LengthZero)
        {
            SpringDictionary.Add(new Spring(_Mass1Index, _Mass2Index, _LengthZero));
            return (SpringDictionary.Count() - 1);
        }
        public int AddJoint(int _Mass1Index, int _Mass2Index, int _MassCentreIndex, double _Angle)
        {
            JointDictionary.Add(new Joint(_Mass1Index, _Mass2Index, _MassCentreIndex, _Angle));
            return (JointDictionary.Count() - 1);
        }
        public class PointMass
        {
            public Tuple<double, double> Position = new Tuple<double, double>(0, 0);
            public Tuple<double, double> Velocity = new Tuple<double,double>(0,0);
            public Tuple<double, double> Acceleration = new Tuple<double, double>(0, 0);
            public Tuple<double, double> Force = new Tuple<double, double>(0, 0);
            public Tuple<double, double> VelocityG = new Tuple<double, double>(0, 0);
            public Tuple<double, double> AccelerationG = new Tuple<double, double>(0, 0);
            public Tuple<double, double> ForceG = new Tuple<double, double>(0, 0);

            public double SpringStiffness = 10000;
            public double SpringDamping = 10000;
            public double LengthZero;
            public double LengthPrevious;
            public double FrictionCoefficient = 10;
            public List<double> NormalForces = new List<double>();


            public double Mass;
            public PointMass(Tuple<double,double> _Position, double _Mass)
            {
                Position = _Position;
                Mass = _Mass;
            }
            public void SetToGForce()
            {
               // Force = new Tuple<double, double>(0, 0);
               // Acceleration = new Tuple<double, double>(0, 0);
                Force = new Tuple<double, double>(0, -Mass * 9.81);
                Acceleration = new Tuple<double, double>(0, -9.81);
                //Force = new Tuple<double, double>(0, 0);
                //Acceleration = new Tuple<double, double>(0, 0);
                if (Position.Item2 < 0)
                {
                    if (LengthPrevious < 0)
                    {
                        LengthPrevious = 0;
                    }
                    Force = TupleAdd(Force, new Tuple<double, double>(0, (-Position.Item2 - (LengthPrevious + Position.Item2)*SpringDamping) * SpringStiffness));
                    //System.Console.WriteLine("Force: " + (-Position.Item2 - (LengthPrevious + Position.Item2) * SpringDamping) * SpringStiffness);
                    LengthPrevious = -Position.Item2;
                }
                else
                {
                    LengthPrevious = -1;
                    
                }
                
                //System.Console.WriteLine("Acc: " + Acceleration.Item1 + " - " + Acceleration.Item2);
            }
            private void CreateAcceleration()
            {
                /*if (Position.Item2 < 0)
                {
                    NormalForces.Add(Force.Item1);
                    while (NormalForces.Count > 10000)
                    {
                        NormalForces.RemoveAt(0);
                    }
                    //System.Console.WriteLine("p, F, FN: " + Position.Item1+ " - " + Force.Item1 + " - " + FrictionCoefficient * -Position.Item2 * SpringStiffness);
                    double NormalForcesSum = 0;
                    foreach(double dou in NormalForces)
                    {
                        NormalForcesSum += dou;
                    }
                    //System.Console.WriteLine("p, F, FN, total: " + Position.Item1 + " - " + NormalForcesSum / NormalForces.Count + " - " + FrictionCoefficient * NormalForcesSum / NormalForces.Count + " - t " + NormalForces.Count);
                    if (Force.Item1 > 0)
                    {
                        if (NormalForcesSum/NormalForces.Count > (-Position.Item2 - (LengthPrevious + Position.Item2) * SpringDamping) * SpringStiffness)
                        {
                            //Force = TupleAdd(Force, new Tuple<double, double>(-FrictionCoefficient * -Position.Item2 * SpringStiffness, 0));
                        }
                        else
                        {
                            Force = new Tuple<double, double>(0, Force.Item2);
                        }
                    }
                    else
                    {
                        if (NormalForcesSum / NormalForces.Count < (-Position.Item2 - (LengthPrevious + Position.Item2) * SpringDamping) * SpringStiffness)
                        {
                            //Force = TupleAdd(Force, new Tuple<double, double>(FrictionCoefficient * -Position.Item2 * SpringStiffness, 0));
                        }
                        else
                        {
                            Force = new Tuple<double, double>(0, Force.Item2);
                        }
                    }
                    Force = new Tuple<double, double>(0, Force.Item2);
                }
                else
                {
                    NormalForces = new List<double>();
                }*/

                //AccelerationG = new Tuple<double, double>(ForceG.Item1 / Mass, ForceG.Item2 / Mass);
                Acceleration = new Tuple<double, double>(Force.Item1 / Mass, Force.Item2 / Mass);
                //System.Console.WriteLine("Ac2: " + Acceleration.Item1 + " - " + Acceleration.Item2);
            }
            private void CreateVelocity(double TimeStep)
            {
                //System.Console.WriteLine("Vel: " + Velocity.Item1 + " - " + Velocity.Item2);
                //VelocityG = TupleAdd(VelocityG, new Tuple<double, double>(AccelerationG.Item1 * TimeStep, AccelerationG.Item2 * TimeStep));
                Velocity = TupleAdd(Velocity, new Tuple<double, double>(Acceleration.Item1 * TimeStep, Acceleration.Item2 * TimeStep));
                //VelocityG = TupleTimesScalar(VelocityG, 0.9999);
                Velocity = TupleTimesScalar(Velocity, 0.9999);
                //Velocity = TupleAdd(Velocity, VelocityG);
                //System.Console.WriteLine("Vel: " + Velocity.Item1 + " - " + Velocity.Item2);
                if (Position.Item2 < 0)
                {
                    //Position = new Tuple<double, double>(Position.Item1, 0);
                    
                    Velocity = new Tuple<double, double>(Velocity.Item1 * 0.99, Velocity.Item2);
                }
                
                //System.Console.WriteLine("Vel: " + Velocity.Item1 + " - " + Velocity.Item2);
            }
            public void CreatePosition(double TimeStep)
            {
                CreateAcceleration();
                CreateVelocity(TimeStep);
                Position = TupleAdd(Position, new Tuple<double, double>(Velocity.Item1 * TimeStep, Velocity.Item2 * TimeStep));
                //System.Console.WriteLine("Pos: " + Position.Item1 + " - " + Position.Item2);
            }
        }
        public class Spring
        {
            public double SpringStiffness = 10000;
            public double SpringDamping = 10000;
            public int Mass1Index;
            public int Mass2Index;
            public double LengthZero;
            public double LengthPrevious;
            public Spring(int _Mass1Index, int _Mass2Index, double _LengthZero)
            {
                Mass1Index = _Mass1Index;
                Mass2Index = _Mass2Index;
                LengthZero = _LengthZero;
                LengthPrevious = LengthZero;
            }
            public double GetLength(List<PointMass> PointMassDictionary)
            {
                return (PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position));
            }
            public Tuple<double, double> ForceOnMass1(List<PointMass> PointMassDictionary)
            {
                return (
                    TupleTimesScalar(
                        TupleTimesScalar(
                            TupleSubtract(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position), 
                            1/PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position)
                        ),
                        (PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position)
                        - LengthZero
                        - (LengthPrevious - PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position))*SpringDamping) * SpringStiffness
                    ));
            }
            public Tuple<double, double> ForceOnMass2(List<PointMass> PointMassDictionary)
            {
                return (
                    TupleTimesScalar(
                        TupleTimesScalar(
                            TupleSubtract(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position),
                            1/PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position)
                        ),
                        (-1)*(PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position)
                        - LengthZero
                        - (LengthPrevious - PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[Mass1Index].Position)) * SpringDamping) * SpringStiffness
                    ));
            }
        }
        public class Joint
        {
            public double JointStiffness = 1000;
            public double JointDamping = 1000;
            public int Mass1Index;
            public int Mass2Index;
            public int MassCentreIndex;
            public double Angle;
            public double AnglePrevious;
            public Joint(int _Mass1Index, int _Mass2Index, int _MassCentreIndex, double _Angle)
            {
                Mass1Index = _Mass1Index;
                Mass2Index = _Mass2Index;
                MassCentreIndex = _MassCentreIndex;
                Angle = _Angle;
            }

            public double GetAngle(List<PointMass> PointMassDictionary)
            {
                return CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position);
            }

            

            public Tuple<double, double> ForceOnMass1(List<PointMass> PointMassDictionary)
            {
                Tuple<double, double> a = TupleSubtract(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position);
                double b = 1 / PointDistance(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position);
                Tuple<double, double> c = TupleTimesScalar(
                                TupleSubtract(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position),
                                1 / PointDistance(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position)
                            );
                Tuple<double, double> d = TupleOrthogonal(
                            TupleTimesScalar(
                                TupleSubtract(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position),
                                1 / PointDistance(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position)
                            )
                        );
                double e = (-1) * (CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)
                        - Angle
                        - (AnglePrevious - CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)) * JointDamping) * JointStiffness / PointDistance(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position);
                double e1 = (-1) * (CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position));
                double e2 = - Angle;
                double e3 = - (AnglePrevious - CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)) * JointDamping;
                double a1 = CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position);
                double aSoll = Angle;
                //System.Console.WriteLine("AngC, AngS: " + a1 + "  " + aSoll);
                double ang1 = CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)
                        - Angle;
                while (ang1 > 180)
                {
                    ang1 -= 360;
                }
                while (ang1 < -180)
                {
                    ang1 += 360;
                }
                double ang2 = AnglePrevious - CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position);
                while (ang2 > 180)
                {
                    ang2 -= 360;
                }
                while (ang2 < -180)
                {
                    ang2 += 360;
                }
                double ang3 = ang1 - ang2 * JointDamping;
                while (ang3 > 180)
                {
                    ang3 -= 360;
                }
                while (ang3 < -180)
                {
                    ang3 += 360;
                }
                if (ang3 > 10)
                {
                    ang3 = 10;
                }
                if (ang3 < -10)
                {
                    ang3 = -10;
                }

                return (
                    TupleTimesScalar(
                        TupleOrthogonal(
                            TupleTimesScalar(
                                TupleSubtract(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position),
                                1 / PointDistance(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position)
                            )
                        ),
                        (-1) * (ang3) * JointStiffness / PointDistance(PointMassDictionary[Mass1Index].Position, PointMassDictionary[MassCentreIndex].Position)
                    ));
            }


            //Einfach Mass1 + Mass2 addieren und minus dran...
            public Tuple<double, double> ForceOnMassCentre(List<PointMass> PointMassDictionary)
            {
                return (TupleTimesScalar(TupleAdd(ForceOnMass1(PointMassDictionary),ForceOnMass2(PointMassDictionary)),-1));
            }

            public Tuple<double, double> ForceOnMass2(List<PointMass> PointMassDictionary)
            {
                double ang1 = CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)
                        - Angle;
                while (ang1 > 180)
                {
                    ang1 -= 360;
                }
                while (ang1 < -180)
                {
                    ang1 += 360;
                }
                double ang2 = AnglePrevious - CalculateAngle(PointMassDictionary[Mass1Index].Position, PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position);
                while (ang2 > 180)
                {
                    ang2 -= 360;
                }
                while (ang2 < -180)
                {
                    ang2 += 360;
                }
                double ang3 = ang1 - ang2 * JointDamping;
                while (ang3 > 180)
                {
                    ang3 -= 360;
                }
                while (ang3 < -180)
                {
                    ang3 += 360;
                }
                if (ang3 > 10)
                {
                    ang3 = 10;
                }
                if (ang3 < -10)
                {
                    ang3 = -10;
                }

                return (
                    TupleTimesScalar(
                        TupleOrthogonal(
                            TupleTimesScalar(
                                TupleSubtract(PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position),
                                1 / PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)
                            )
                        ),
                        (-1) * (ang3) * JointStiffness / PointDistance(PointMassDictionary[Mass2Index].Position, PointMassDictionary[MassCentreIndex].Position)
                    ));
            }
        }

        public void Initialize()
        {
            foreach (Spring spring in SpringDictionary)
            {
                spring.LengthPrevious = spring.GetLength(PointMassDictionary);
            }//Do the same for the joints
            foreach (Joint joint in JointDictionary)
            {
                joint.AnglePrevious = joint.GetAngle(PointMassDictionary);
            }
        }

        public void SetAngles(double[] Angles)
        {
            int i = 0;
            foreach (Joint joint in JointDictionary)
            {
                joint.Angle = Angles[i];
                i++;
            }
        }
        
        public void Update(double TimeStep)
        {
            //System.Console.WriteLine("Update!");
            foreach (PointMass pointMass in PointMassDictionary)
            {
                pointMass.SetToGForce();
            }
            foreach (Spring spring in SpringDictionary)
            {
                PointMassDictionary[spring.Mass1Index].Force = TupleAdd(PointMassDictionary[spring.Mass1Index].Force, spring.ForceOnMass1(PointMassDictionary));
                PointMassDictionary[spring.Mass2Index].Force = TupleAdd(PointMassDictionary[spring.Mass2Index].Force, spring.ForceOnMass2(PointMassDictionary));
            }
            foreach (Joint joint in JointDictionary)
            {
                //System.Console.WriteLine("Angle: " + joint.GetAngle(PointMassDictionary));
                //System.Console.WriteLine("Force on 1: " + joint.ForceOnMass1(PointMassDictionary));
                PointMassDictionary[joint.Mass1Index].Force = TupleAdd(PointMassDictionary[joint.Mass1Index].Force, joint.ForceOnMass1(PointMassDictionary));
                PointMassDictionary[joint.Mass2Index].Force = TupleAdd(PointMassDictionary[joint.Mass2Index].Force, joint.ForceOnMass2(PointMassDictionary));
                PointMassDictionary[joint.MassCentreIndex].Force = TupleAdd(PointMassDictionary[joint.MassCentreIndex].Force, joint.ForceOnMassCentre(PointMassDictionary));
                //ToDo Calculate Forces on Joint Points
            }

            foreach (Spring spring in SpringDictionary)
            {
                spring.LengthPrevious = spring.GetLength(PointMassDictionary);
            }//Do the same for the joints
            foreach (Joint joint in JointDictionary)
            {
                joint.AnglePrevious = joint.GetAngle(PointMassDictionary);
            }
            foreach (PointMass pointMass in PointMassDictionary)
            {
                pointMass.CreatePosition(TimeStep);
            }
        }

        public Bitmap Draw()
        {
            Bitmap Drawn = new Bitmap(WorldWidth, WorldHeight);
            Graphics G = Graphics.FromImage(Drawn);

            Pen blackPen = new Pen(Color.Black, 3);
            int offset = 50;
            G.DrawRectangle(blackPen, new Rectangle(offset, offset, WorldWidth - 2*offset, WorldHeight - 2*offset));
            foreach (PointMass pm in PointMassDictionary)
            {
                G.DrawEllipse(blackPen, offset + Convert.ToInt32(Math.Round(pm.Position.Item1 * (WorldWidth - 2 * offset) / WorldWidthReal))-3, WorldHeight - offset - Convert.ToInt32(Math.Round(pm.Position.Item2 * (WorldHeight - 2 * offset) / WorldHeightReal))-3, 6, 6);
            }
            foreach (Spring spring in SpringDictionary)
            {
                PointMass pm = PointMassDictionary[spring.Mass1Index];
                Point px = new Point(offset + Convert.ToInt32(Math.Round(pm.Position.Item1 * (WorldWidth - 2 * offset) / WorldWidthReal)), WorldHeight - offset - Convert.ToInt32(Math.Round(pm.Position.Item2 * (WorldHeight - 2 * offset) / WorldHeightReal)));
                pm = PointMassDictionary[spring.Mass2Index];
                Point py = new Point(offset + Convert.ToInt32(Math.Round(pm.Position.Item1 * (WorldWidth - 2 * offset) / WorldWidthReal)), WorldHeight - offset - Convert.ToInt32(Math.Round(pm.Position.Item2 * (WorldHeight - 2 * offset) / WorldHeightReal)));
                G.DrawLine(blackPen, px, py);
            }
            return Drawn;
        }

        static double PointDistance(Tuple<double, double> p1, Tuple<double, double> p2)
        {
            return Math.Sqrt(Math.Pow(p1.Item1 - p2.Item1, 2) + Math.Pow(p1.Item2 - p2.Item2, 2));
        }
        static double CalculateAngle(Tuple<double, double> p1, Tuple<double, double> p2, Tuple<double, double> pc)
        {
            double val = 360.0 / Math.PI * (Math.Atan2(p1.Item2 - pc.Item2, p1.Item1 - pc.Item1) - Math.Atan2(p2.Item2 - pc.Item2, p2.Item1 - pc.Item1));
            while (val < 0)
            {
                val += 360;
            }
            while (val > 360)
            {
                val -= 360;
            }
            return val;
        }

        public static Tuple<double, double> TupleAdd(Tuple<double, double> c1, Tuple<double, double> c2)
        {
            return new Tuple<double, double>(c1.Item1 + c2.Item1, c1.Item2 + c2.Item2);
        }
        public static Tuple<double, double> TupleSubtract(Tuple<double, double> c1, Tuple<double, double> c2)
        {
            return new Tuple<double, double>(c1.Item1 - c2.Item1, c1.Item2 - c2.Item2);
        }
        public static Tuple<double, double> TupleTimesScalar(Tuple<double, double> c1, double c2)
        {
            return new Tuple<double, double>(c1.Item1 * c2, c1.Item2 * c2);
        }

        public static Tuple<double,double> TupleOrthogonal(Tuple<double, double> c1)
        {
            return new Tuple<double, double>(-c1.Item2, c1.Item1);
        }
    }

    public class GeneticAlgorithm
    {
        int _WorldWidth;
        int _WorldHeight;
        double _WorldWidthReal;
        double _WorldHeightReal;
       

        public class Individual : ICloneable
        {
            public double[][] Angles;
            public double Fitness;
            public Adam Adam1;

            int _WorldWidth;
            int _WorldHeight;
            double _WorldWidthReal;
            double _WorldHeightReal;

            public Individual(int length, int nrjoints, int _WorldWidth, int _WorldHeight, double _WorldWidthReal, double _WorldHeightReal)
            {
      
                Angles = new double[length][];
                for (int i = 0; i < length; i++)
                {
                    Angles[i] = new double[nrjoints];
                }

                // Adam1 = new Adolf(_WorldWidth, _WorldHeight, _WorldWidthReal, _WorldHeightReal);
            }

            public object Clone()
            {
                Individual Cloned = new Individual(Angles.Length, Angles[0].Length, _WorldWidth, _WorldHeight, _WorldWidthReal, _WorldHeightReal);
                Cloned.Angles = new double[Angles.Length][];
                for (int i = 0; i < Angles.Length; i++)
                {
                    Cloned.Angles[i] = new double[Angles[0].Length];
                }
                for (int i = 0; i < Angles.Length; i++)
                {
                    for (int j = 0; j < Angles[0].Length; j++)
                    {
                        Cloned.Angles[i][j] = Angles[i][j];
                    }
                }
                return Cloned;
            }
        }

        double CrossoverFrequency = 0.6;
        double MutationFrequency = 0.1;
        int PopulationSize = 10;
        double ElitistFrequency = 0.1;

        Individual[] Population;
        int Length;
        int Time;
        int NrJoints;

        Individual Best = null;

 

        public GeneticAlgorithm()
        {

        }

        // length = number of time steps
        // time = how many milli seconds are one time step
        public void Go(int length, int time, int nrjoints, int _WorldWidth,
            int _WorldHeight,
            double _WorldWidthReal,
            double _WorldHeightReal, Form1.aufruf Delegate)
        {
            Population = new Individual[PopulationSize];
            Length = length * 1000 / time;
            NrJoints = nrjoints;
            Time = time;

            for (int i = 0; i < PopulationSize; i++)
            {
                Population[i] = new Individual(Length, nrjoints, _WorldWidth,
            _WorldHeight,
            _WorldWidthReal,
            _WorldHeightReal);
            }

            Initialize();

            int Counter = 0;
            while (true)
            {
                Breed();
                if (Counter % 1 == 0)
                    Delegate(Best.Angles, Length, Time, Counter++);
            }
        }

        public void Initialize()
        {
            Random Rnd = new Random();
            for (int i = 0; i < PopulationSize; i++)
            {
                for (int j=0;j<Length;j++) {
                    for (int m=0;m<NrJoints;m++) {
                        Population[i].Angles[j][m] = Rnd.Next(360);
                    }
                }
               

            }
        }

        public void Breed()
        {
            double[] SummedFitness = new double[PopulationSize];

            for (int i = 0; i < PopulationSize; i++)
            {
                Population[i].Fitness = CalculateFitness(Population[i]);
                if (Best == null || Population[i].Fitness > Best.Fitness)
                    Best = Population[i];
                SummedFitness[i] += Population[i].Fitness;
            }

            Individual[] NewPopulation = new Individual[PopulationSize];

            // take best individuals from last population
            for (int i = 0; i < Math.Ceiling(PopulationSize * ElitistFrequency); i++)
            {
                NewPopulation[i] = (Individual)Population[i].Clone();
            }

            Random Rnd = new Random();

            for (int i = (int)Math.Ceiling(PopulationSize * ElitistFrequency); i < PopulationSize; i++)
            {
                double RParent1 = Rnd.NextDouble() * SummedFitness[PopulationSize - 1];
                double RParent2 = Rnd.NextDouble() * SummedFitness[PopulationSize - 1];

                Individual Parent1 = null;
                Individual Parent2 = null;
                Individual Child = new Individual(Length, NrJoints, _WorldWidth,
            _WorldHeight,
            _WorldWidthReal,
            _WorldHeightReal);
                // Tournament selection of the parents
                for (int j = 0; j < PopulationSize; j++)
                {
                    if (SummedFitness[j] >= RParent1)
                    {
                        Parent1 = Population[j];
                    }
                    if (SummedFitness[j] >= RParent2)
                    {
                        Parent2 = Population[j];
                    }
                }

                // do a crossover
                Child = (Individual)Parent1.Clone();
                if (Rnd.NextDouble() <= CrossoverFrequency)
                {
                    int COPoint = Rnd.Next(Length);
                    for (int j = COPoint; j < Length; j++)
                    {
                        for (int z = 0; z < NrJoints; z++)
                        {
                            Child.Angles[j][z] = Parent2.Angles[j][z];
                        }
                    }
                }

                for (int j = 0; j < Length; j++)
                {
                    for (int z = 0; z < NrJoints; z++)
                    {
                        // do a mutation
                        if (Rnd.NextDouble() <= MutationFrequency)
                        {
                            Child.Angles[j][z] = Child.Angles[j][z] + Rnd.Next(100) - 50;
                        }
                    }
                }

                NewPopulation[i] = Child;
            }

            Population = NewPopulation;
        }

        public double CalculateFitness(Individual ind)
        {
            // run physics simulation and return distance travelled as fitness measure

            Adam Adam1;
            Adam1 = new Adam(_WorldWidth,
            _WorldHeight,
            _WorldWidthReal,
            _WorldHeightReal);
            /*int p1 = Adam1.AddPointMass(0.1, 1.7, 20);//Head/Shoulders
        int p2 = Adam1.AddPointMass(0, 0.9, 20);//Hip
        Adam1.AddSpring(p1, p2, 0.8);
        int p3 = Adam1.AddPointMass(-0.1, 0.5, 8);//Knee1
        Adam1.AddSpring(p2, p3, 0.4);
        Adam1.AddJoint(p1, p3, p2, 195);//HipKnee1*/

            double XPos = Adam1.SimulateAngles(ind.Angles, Time, false, null);
            return XPos;
        }
    }
}
