import lejos.nxt.SensorPort;
import lejos.nxt.TachoMotorPort;
import weka.core.converters.ConverterUtils.DataSource;
import weka.classifiers.lazy.IBk;
import weka.core.Instances;
import weka.core.Instance;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.InputStreamReader;
import java.util.HashMap;

import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.Color;
import lejos.nxt.Sound;
import lejos.nxt.remote.NXTCommand;
import lejos.pc.comm.NXTComm;
import lejos.pc.comm.NXTCommLogListener;
import lejos.pc.comm.NXTCommandConnector;
import lejos.pc.comm.NXTConnector;
public class KNNTest {
	static ColorSensor light;
	static NXTMotor motor;
	static int ANGLE_STEP = 30;
	static int TOTAL_ANGLE = 360;
	static  IBk knn = new IBk();
	static Instance testInstance=null;
	static void initNTX() {
		NXTConnector conn = new NXTConnector();
		/*
		conn.addLogListener(new NXTCommLogListener() {
			public void logEvent(String message) {
				System.out.println(message);				
			}

			public void logEvent(Throwable throwable) {
				System.err.println(throwable.getMessage());			
			}			
		});
		conn.setDebug(true);
		*/
		if (!conn.connectTo("usb://NXT", NXTComm.LCP)) {
			System.err.println("Failed to connect");
			System.exit(1);
		}
		NXTCommandConnector.setNXTCommand(new NXTCommand(conn.getNXTComm()));		
		light = new ColorSensor(SensorPort.S1);
		light.setFloodlight(Color.GREEN);
		MotorPort port =  MotorPort.A;
		motor = new NXTMotor(port);		
		//Motor.A.rotateTo(0);
		Motor.A.setSpeed(100);
	}
	static int[][] scanNumber(int[][] result,int loop) throws Exception{
		//int[][] result = new int[loop][TOTAL_ANGLE/5];
		for(int l=0;l<loop;l++) {
			//Motor.A.rotateTo(0);			
		   for(int a=0,index=0;a<TOTAL_ANGLE;a+=ANGLE_STEP,index++) {
		     Motor.A.rotate(ANGLE_STEP);
		     //System.out.println("Tachometer A: " + Motor.A.getTachoCount());		
		     //System.out.println("light = " + light.getNormalizedLightValue());
		     //Thread.sleep(100);
		     result[l][index]=light.getNormalizedLightValue();
		  }				
		}
		return result;
	}
	
	static double[] scanNumber() throws Exception{
		//Motor.A.rotateTo(0);		
		double[] result = new double[TOTAL_ANGLE/ANGLE_STEP];
        for(int a=0,index=0;a<TOTAL_ANGLE;a+=ANGLE_STEP,index++) {
		     Motor.A.rotate(ANGLE_STEP);
		     //System.out.println("Tachometer A: " + Motor.A.getTachoCount());	
		     int value = light.getNormalizedLightValue();
		     //System.out.print( value+",");
		     //Thread.sleep(100);
		     result[index]=(double)value;
		}
		return result;
	}	
	
	static void writeDataFile(String fileName,int[][][] scanData) throws Exception{
        File file = new File(fileName);
        FileWriter fw = new FileWriter(file,false);
        fw.write("@relation 'numberLight'\n");        
        fw.write("@attribute vendor {0,1,2,3,4,5,6,7,8,9}\n");
        for(int i=0;i<TOTAL_ANGLE;i+=ANGLE_STEP)
        fw.write("@attribute light_angle"+i+" real\n");
        fw.write("\n@data\n");
        for(int loop=0;loop<scanData[0].length;loop++) {
        	for(int n=0;n<10;n++) {        	
        	    fw.write(String.valueOf(n));
        	    for(int a=0,index=0;a<TOTAL_ANGLE;a+=ANGLE_STEP,index++) {
                    if (a % TOTAL_ANGLE == (TOTAL_ANGLE-ANGLE_STEP)) 
                    	fw.write(","+String.valueOf(scanData[n][loop][index])+"\n");
                    else	
        	    	    fw.write(","+String.valueOf(scanData[n][loop][index]));
        	    }
        	}    
        }
        fw.close();
	}

	static double[] trainData(String fileName,int percentage) throws Exception {
        DataSource source = new DataSource(fileName);
        Instances data = source.getDataSet();  
        testInstance = data.firstInstance();
        
       //knn.setKNN(k);
       // knn.setWindowSize(30);
       
        data.setClassIndex(0);
        //Instances train = new Instances(data, 0, data.numInstances() / 2);
        //Instances test = new Instances(data, data.numInstances() / 2, data.numInstances() - data.numInstances() / 2);
        int totalData =data.numInstances();
        int totalTrain = totalData*percentage/100;
        Instances train = new Instances(data, 0, totalTrain);
        Instances test = new Instances(data, totalTrain, totalData-totalTrain);
        
        int prob=0;
        int optiK = 3;
        for(int k=1;k< Math.sqrt(train.numInstances()) && k<=20;k++) {
        	knn.setKNN(k);        	
            knn.buildClassifier(train);
            int count =0;
            for (int i = 0; i < test.numInstances(); i++) {
            	//System.out.print("test:"+i+" "+test.instance(i));
   		        //System.out.println(" 辨識="+knn.classifyInstance(test.instance(i))+" 實際=" + test.instance(i).classValue());		
                if (knn.classifyInstance(test.instance(i)) == test.instance(i).classValue())
                    count++;
            }	
            System.out.println("   K 值為:"+k+" 正確率="+100.0*count/test.numInstances()+"%");     
            if (count > prob) {
                optiK = k;
                prob = count;
            }     
        }        
        double result[] = new double[2];
        result[0] = optiK;
        result[1] = 100.0*prob/test.numInstances();
        return result;
	}	
	
	//static HashMap<Integer,int[][]> scanData = new HashMap<Integer,int[][]>();
		
    public static void main(String[] args) throws Exception {
    	initNTX();    	
    	String inputS;    	
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.println("開始數字辨識作業: ");        
        System.out.println("STEP 1 建立數字特徵資料:  後續請依指示依序放入0-9 數字牌");
        System.out.print("  首先請先輸入每個數字要掃描的角度:"); 
        inputS = br.readLine();
        ANGLE_STEP = Integer.parseInt(inputS);
        System.out.print("  接下來輸入每個數字要掃描的次數，0表示直接讀取之前掃描資料:"); 
        inputS = br.readLine();
        int scanLoop = Integer.parseInt(inputS);
        if(scanLoop!=0) {
        int[][][] dataPool = new int[10][scanLoop][TOTAL_ANGLE/ANGLE_STEP];
        for(int i=0;i<10;i++) {
        	System.out.print("   現在請放置數字 "+i+" 數字牌，放好後請按Enter 鍵");
        	inputS = br.readLine();
        	// 紀錄掃描資料
        	int[][] data = scanNumber(dataPool[i],scanLoop);        	        	
        }        
        System.out.println("數字特徵資料蒐集完成:");        
        System.out.println("STEP 2  進行資料整理與資料檔輸出作業...");
        try {
            writeDataFile("data_"+ANGLE_STEP+".arff",dataPool) ;
            System.out.println("完成資料整理與資料檔輸出作業");            
        }  catch(Exception e) {
            e.printStackTrace();
            System.out.println("資料整理與資料檔輸出錯誤");               
        }    	
        }
        
        System.out.print("STEP 3 掃描資料將分為訓練資料與測試資料，請輸入訓練資料占比:");        
        inputS = br.readLine();
        int percentage = Integer.parseInt(inputS);        

        double[] result = trainData("data_"+ANGLE_STEP+".arff",percentage);
        System.out.println("   完成模型訓練，最佳 K 值為:"+result[0]+"正確率="+result[1]+"%");
        System.out.print("   請輸入要使用的K值:");        
        inputS = br.readLine();
        int k = Integer.parseInt(inputS);       
        knn.setKNN(k);
        
        while(true) {
        System.out.print("STEP 4 請放入欲辨識的數字牌，放好後請按Enter 鍵，離開請按q");       
        inputS = br.readLine();
        if(inputS.equals("q"))
        	return;
        double[] testData = scanNumber();System.out.println("");
        //testData = scanNumber();System.out.println("");     
        testInstance =(Instance) testInstance.copy();
        for(int i=0;i<testData.length;i++)
        testInstance.setValueSparse(i+1, testData[i]);
        testInstance.setClassValue("1");
        System.out.println("辨識結果為"+(int)knn.classifyInstance(testInstance));
        }
    }  
    
}	
