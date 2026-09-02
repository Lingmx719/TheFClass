
// AutoRobotDoc.h : CAutoRobotDoc 类的接口
//


#pragma once

#define DATA_BUFFER_LEN 800//数据缓冲区长度 = 800，数组最大存 800 组仿真数据
#include<vector>

//单帧仿真记录，用于CSV保存&回放
struct SimRecord
{
	double time;
	//真实机器人
	double robotX;
	double robotY;
	double robotTheta;
	//传感器
	double gpsX;
	double gpsY;
	double insX;
	double insY;
	//EKF融合
	double ekfX;
	double ekfY;
	double ekfTheta;
	//控制输出
	double resV;
	double resW;
	//误差
	double errGps;
	double errEKF;
};

class CAutoRobotDoc : public CDocument
{
protected: // 仅从序列化创建
	CAutoRobotDoc();
	DECLARE_DYNCREATE(CAutoRobotDoc)

// 特性
public:
	//定义结构体
	struct CObs
	{
		double x;
		double y;
		double z;
	};
	//机器人状态
	double RobotX,RobotY,RobotTheta;
	double RobotV,RobotW;
	//起终点状态
	double StartX,StartY;
	double FinX,FinY;
	//GPS
	double GPSNoise;
	double GPSX,GPSY;
	double GPSWeight;
	//INS
	double INSNoise;
	double INSX,INSY;
	double INSWeight;
	void Sensor();
	//加权平均融合
	double WeightX,WeightY;
	void Weight();
	//扩展卡尔曼滤波EKF
	double ekfX,ekfY,ekfTheta;
	double P[3][3];//协方差矩阵 3×3，代表状态[x,y,θ]的不确定度、误差
	double Q[3][3];//过程噪声矩阵 3×3，机器人运动本身带来的噪声
	double R[2][2];//观测噪声矩阵 2×2，传感器（GPS / 定位）测量噪声
	double H[2][3];//观测矩阵 2 行 3 列，状态转观测的雅克比矩阵
	double K[3][2];//卡尔曼增益 3×2，EKF 核心，决定相信模型还是相信传感器
	double InnovX,InnovY;// InnovX：X 方向观测残差（测量值 - 预测值） InnovY：Y 方向观测残差
	double S[2][2];//残差协方差矩阵，用于计算卡尔曼增益
	void EKF();//函数：执行一次扩展卡尔曼滤波计算
	//人工势场避障
	double attract,repel;//repel:斥力大小，障碍物对机器人排斥力
	double EdgeDis;//障碍物边界距离，小于这个距离就产生斥力
	double GradientStep;//梯度下降步长，势场求解迭代步长
	double AttractX,AttractY;//引力在 X、Y 方向分量
	double RepelX,RepelY;
	double TotalX,TotalY;//总分量
	double ExpectVX,ExpectVY;//期望速度 X、Y，由势场合力换算出来机器人期望速度
	void ObsAvoid();//避障算法函数，ObsAvoid 障碍物避障
	//PID
	double kp,ki,kd;//PID 三个参数：比例、积分、微分
	double dV;//速度误差，目标速度 - 当前实际速度 
	double TotalDV;//积分项，误差累加（对应 ki）
	double LastDV;//上一时刻的速度误差，用来算微分项 kd
	double resV,resW;//resV：输出线速度  resW：输出角速度
	void PID();
	//激光雷达
	double GPSFreq;//GPS频率
	double INSFreq;//INS频率
	int LineNum;//激光雷达线束数量，多少个激光点
	double MaxDis,LineDis;//MaxDis：雷达最大探测距离  LineDis：某一束激光测量距离
	double NoiseProb;//噪声概率,模拟激光雷达出现噪声、异常跳点的概率
	void SimulateLaser();//雷达函数
    std::vector<double>LaserDist;//雷达距离数组，对应每一条激光射线的距离
	//地图绘制
	int DrawMode;
	double ObsX,ObsY,ObsR;//Obstacle 障碍物,ObsX：障碍物圆心 X 坐标,ObsY：障碍物圆心 Y 坐标
	std::vector<CObs> m_obsList; //新增多障碍物列表
	
	bool IfRun;//仿真运行开关
	//轨迹
	double TagX[2000],TagY[2000];//迹点坐标数组
	int TagNum;//已经存了多少个轨迹点，数组计数器。每走一步TagNum++
	int ReTag;//轨迹刷新标记
	void ClearTag();//函数，清空全部轨迹数组，把TagNum清零，清除历史路径
	void RobotSta(double dt);//机器人运动状态
	void Step(double dt);//步伐
	double Random(double m,double d);//噪声随机数
	void InitAllParam();//初始化
	void SimStep();//停止
	
	int NaviMode;//导航模式：0单GPS，1单INS，2 GPS-INS融合
	int FusionMode;//融合算法：0->加权平均，1->EKF
	double FusionX;//融合结果X坐标
	double FusionY;//融合结果Y坐标
	int AvoidMode;//避障算法：0人工势场

	
	double ErrGps[DATA_BUFFER_LEN];//GPS 原始定位的误差数组，记录每一步 GPS 的误差
	double ErrEKF[DATA_BUFFER_LEN];//EKF 滤波之后的定位误差数组，对比 GPS，看滤波效果
	double ErrIns[DATA_BUFFER_LEN];//INS 定位误差数组（新增），用于示波器显示 INS 精度随时间的变化
	double PidVData[DATA_BUFFER_LEN];//PID 输出的速度 V 历史数组，保存每一步 PID 输出线速度
	int DataIndex;//数组下标计数器。每仿真一步,DataIndex++，往数组的这个位置存入一组数据

	bool IsPlayBack;//是否回放模式
	int SimFreq; //仿真刷新频率，定时器ms

	//保存回放
	void SaveSimData(CString filePath);//保存仿真数据函数
	void LoadPlayBack(CString filePath);//加载回放
	void CalcError();//计算误差函数

	//进阶需求：差速轮机器人
	double WheelBase; //轮距
	double vl, vr;    //左右轮转速

	//=====新增回放相关=====
	std::vector<SimRecord> m_records;
	bool        m_isReplay;
	int         m_replayIdx;
	double      m_simTime;

	//函数声明
	void AddRecord();
	void ApplyReplayStep();
	BOOL SaveToCSV(CString filePath);
	BOOL LoadFromCSV(CString filePath);
// 操作
public:

// 重写
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// 实现
public:
	virtual ~CAutoRobotDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// 用于为搜索处理程序设置搜索内容的 Helper 函数
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
};
