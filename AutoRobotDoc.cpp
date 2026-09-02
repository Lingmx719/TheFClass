// AutoRobotDoc.cpp : CAutoRobotDoc ���ʵ��
//

#include "stdafx.h"
// SHARED_HANDLERS ������ʵ��Ԥ��������ͼ������ɸѡ�������
// ATL ��Ŀ�н��ж��壬�����������Ŀ�����ĵ����롣
#include <string>
#ifndef SHARED_HANDLERS
#include "AutoRobot.h"
#endif

#include "AutoRobotDoc.h"

#include <propkey.h>
#include <cmath>
#include<vector>
#include <cstring>
#include <ctime>
#include <fstream>
using namespace std;
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CAutoRobotDoc

IMPLEMENT_DYNCREATE(CAutoRobotDoc, CDocument)

BEGIN_MESSAGE_MAP(CAutoRobotDoc, CDocument)
END_MESSAGE_MAP()


// CAutoRobotDoc ����/����

CAutoRobotDoc::CAutoRobotDoc()
{
	// TODO: �ڴ����һ���Թ������
	DataIndex = 0;

	AttractX = 0.0;
	AttractY = 0.0;
	AvoidMode = 0;
	DrawMode = 0;
	LineNum = 36;
	LaserDist.resize(LineNum);
}

CAutoRobotDoc::~CAutoRobotDoc()
{
}

BOOL CAutoRobotDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: 在此添加重新初始化代码
	// (SDI 文档将重用该文档)
	srand((unsigned int)time(NULL));
	//机器人默认初始位置
	RobotX = 0;
	RobotY = 0;
	RobotTheta = 0;
	//起终点默认位置
	StartX = 0;
	StartY = 0;
	FinX = 20;
	FinY = 20;
	//GPS初始值
	GPSNoise = 0.15;
	GPSWeight = 0.5;
	GPSX = 0;
	GPSY = 0;
	//INS初始值
	INSNoise = 0.08;
	INSWeight = 0.5;
	INSX = 0;
	INSY = 0;
	//EKF初始值
	ekfX = RobotX;
	ekfY = RobotY;
	ekfTheta = RobotTheta;
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] = 0;
	P[0][0] = 0.01;
	P[1][1] = 0.01;
	P[2][2] = 0.01;

	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Q[i][j] = 0;
	Q[0][0] = 0.01;
	Q[1][1] = 0.01;
	Q[2][2] = 0.001;

	for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) R[i][j] = 0;
	R[0][0] = 0.02;
	R[1][1] = 0.02;

	H[0][0] = 1; H[0][1] = 0; H[0][2] = 0;
	H[1][0] = 0; H[1][1] = 1; H[1][2] = 0;
	//人工势场
	attract = 2.0;
	repel = 70.0;
	EdgeDis = 1.4;
	GradientStep = 0.12;
	RepelX = 0.0;
	RepelY = 0.0;
	TotalX = 0.0;
	TotalY = 0.0;
	ExpectVX = 0.0;
	ExpectVY = 0.0;

	//地图绘制
	DrawMode = 0;
	ObsX = 10;
	ObsY = 10;
	ObsR = 2;
	IfRun = false;
	//轨迹
	ClearTag();

	//PID参数初始化
	kp = 1.2;
	ki = 0.05;
	kd = 0.1;
	//PID内部变量清零
	TotalDV = 0;
	LastDV = 0;
	RobotV = 0;
	RobotW = 0;
	//算法
	FusionMode = 0; //默认加权平均
	FusionX = 100.0;
	FusionY = 100.0;
	NaviMode = 0;		//默认打开：单GPS
	// 激光雷达初始化
	//传感器参数
	GPSFreq = 10.0;
	GPSNoise = 0.15;
	INSFreq = 100.0;
	INSNoise = 0.08;
	LineNum = 36;
	MaxDis = 8.0;
	NoiseProb = 0.05;
	LaserDist.assign(LineNum, MaxDis);
	m_obsList.clear();
	//把默认紫色障碍物加入避障列表！！
	CObs tempObs;
	tempObs.x = ObsX;
	tempObs.y = ObsY;
	tempObs.z = ObsR;
	m_obsList.push_back(tempObs);

	//====新增重置回放状态====
	m_isReplay = false;
	m_replayIdx = 0;
	m_simTime = 0.0;
	m_records.clear();
	return TRUE;
}

//������
void CAutoRobotDoc::Sensor()
{
	GPSX = RobotX + Random(0, GPSNoise);
	GPSY = RobotY + Random(0, GPSNoise);

	INSX = RobotX + Random(0, INSNoise);
	INSY = RobotY + Random(0, INSNoise);
}

//加权平均融合
void CAutoRobotDoc::Weight()
{
	double sumW = GPSWeight + INSWeight;
	WeightX = (GPSX * GPSWeight + INSX * INSWeight) / sumW;
	WeightY = (GPSY * GPSWeight + INSY * INSWeight) / sumW;
}

//EKF
void CAutoRobotDoc::EKF()
{
	// ===== 新增：预测步骤（先做）=====
	double dt = 0.05; // 固定时间步长
	double v = resV;
	double w = resW;
	double theta = ekfTheta;

	// 状态预测
	if (fabs(w) < 1e-6)
	{
		ekfX += v * cos(theta) * dt;
		ekfY += v * sin(theta) * dt;
	}
	else
	{
		ekfX += (v / w) * (sin(theta + w * dt) - sin(theta));
		ekfY += (v / w) * (-cos(theta + w * dt) + cos(theta));
		ekfTheta += w * dt;
	}

	// P 矩阵预测（增加不确定度）
	P[0][0] += Q[0][0];
	P[1][1] += Q[1][1];
	P[2][2] += Q[2][2];

	InnovX = GPSX - ekfX;
	InnovY = GPSY - ekfY;

	S[0][0] = H[0][0] * P[0][0] * H[0][0] + R[0][0];
	S[0][1] = 0;
	S[1][0] = 0;
	S[1][1] = H[1][1] * P[1][1] * H[1][1] + R[1][1];

	double S00_inv = 1.0 / S[0][0];
	double S11_inv = 1.0 / S[1][1];

	K[0][0] = P[0][0] * H[0][0] * S00_inv;
	K[0][1] = P[0][1] * H[1][1] * S11_inv;
	K[1][0] = P[1][0] * H[0][0] * S00_inv;
	K[1][1] = P[1][1] * H[1][1] * S11_inv;
	K[2][0] = P[2][0] * H[0][0] * S00_inv;
	K[2][1] = P[2][1] * H[1][1] * S11_inv;

	ekfX = ekfX + K[0][0] * InnovX + K[0][1] * InnovY;
	ekfY = ekfY + K[1][0] * InnovX + K[1][1] * InnovY;
	ekfTheta = ekfTheta + K[2][0] * InnovX + K[2][1] * InnovY;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double I_KH = ((i == j) ? 1.0 : 0.0) - (K[i][0] * H[0][j] + K[i][1] * H[1][j]);
			P[i][j] = I_KH * P[i][j];
		}
	}
}

void CAutoRobotDoc::SimStep()//调试打印，输出参数，看对话框传过来的噪声等有没有生效
{
	// 如果是回放模式，直接返回，不跑仿真算法！
	if (m_isReplay)
		return;

	const double dt = 0.05;
	m_simTime += dt;
	SimulateLaser();
	Sensor();
	Weight();
	PID();
	ObsAvoid();
	RobotSta(dt); //时间步长，定时器50ms
	//对话框单选按钮
	
	//对话框单选按钮
	if (NaviMode == 0)
	{
		//单GPS模式，直接拿GPS观测
		FusionX = GPSX;
		FusionY = GPSY;
	}
	else if (NaviMode == 1)
	{
		//单INS模式，直接拿INS观测
		FusionX = INSX;
		FusionY = INSY;
	}
	else if (NaviMode == 2)
	{
		//GPS‑INS融合，看下拉框选的FusionMode
		if (FusionMode == 0)
		{
			//加权平均
			FusionX = 0.5 * GPSX + 0.5 * INSX;
			FusionY = 0.5 * GPSY + 0.5 * INSY;
		}
		else if (FusionMode == 1)
		{
			//EKF滤波算法
			EKF();
			FusionX = ekfX;
			FusionY = ekfY;
		}
	}
	//保存轨迹点，用于视图绘制轨迹
	if (TagNum < 2000)
	{
		TagX[TagNum] = RobotX;
		TagY[TagNum] = RobotY;
		TagNum++;
	}

	CalcError(); //更新示波器误差数据
	//====新增：保存当前完整帧记录====
	AddRecord();

	//NaN防护，防止参数过大机器人消失
	if (RobotX != RobotX || RobotY != RobotY || RobotTheta != RobotTheta)
	{
		RobotX = 120;
		RobotY = 120;
		RobotTheta = 0;
		AfxMessageBox(_T("仿真数值异常，机器人位置重置！"));
	}
	//计算机器人到终点的距离
	double distToGoal = sqrt((RobotX - FinX) * (RobotX - FinX) + (RobotY - FinY) * (RobotY - FinY));
	const double stopThreshold = 0.4;	//距离小于0.4，判定到达终点
	if (distToGoal < stopThreshold)
	{
		IfRun = false;
	}
}

//人工势场避障
void CAutoRobotDoc::ObsAvoid()
{
	double dxGoal = FinX - RobotX;
	double dyGoal = FinY - RobotY;
	double distGoal = sqrt(dxGoal * dxGoal + dyGoal * dyGoal);

	double attrX, attrY;
	const double attrMaxDist = 6.0;	//�����þ��룬������С���������ٱ���

	if (distGoal > attrMaxDist)
	{
		//�����յ�Զ��ȡ��λ����������ֵ�̶�
		double unitGx = dxGoal / distGoal;
		double unitGy = dyGoal / distGoal;
		attrX = attract * unitGx * attrMaxDist;
		attrY = attract * unitGy * attrMaxDist;
	}
	else
	{
		//�����յ㣬�ָ�����������������˥��
		attrX = attract * dxGoal;
		attrY = attract * dyGoal;
	}

	// 处理多个障碍物：把所有障碍物的斥力累加
	double repX = 0;
	double repY = 0;

	// 先处理 m_obsList 中的多个障碍物
	double minSurfDist = 1e9;
	for (size_t i = 0; i < m_obsList.size(); ++i)
	{
		double ox = m_obsList[i].x;
		double oy = m_obsList[i].y;
		double orad = m_obsList[i].z;
		double dxObs = RobotX - ox;
		double dyObs = RobotY - oy;
		double distCenter = sqrt(dxObs * dxObs + dyObs * dyObs);
		double distSurf = distCenter - orad;
		if (distSurf < EdgeDis && distSurf > 0.0001)
		{
			double nx = dxObs / distCenter;
			double ny = dyObs / distCenter;
			double temp = repel * (1.0 / distSurf - 1.0 / EdgeDis);
			repX += temp * nx;
			repY += temp * ny;
			if (distSurf < minSurfDist) minSurfDist = distSurf;
		}
	}

	// 兼容旧的单障碍物设置（如果用户只用了旧变量）
	{
		double dxObs = RobotX - ObsX;
		double dyObs = RobotY - ObsY;
		double distCenter = sqrt(dxObs * dxObs + dyObs * dyObs);
		double distSurf = distCenter - ObsR;
		if (distSurf < EdgeDis && distSurf > 0.0001)
		{
			double nx = dxObs / distCenter;
			double ny = dyObs / distCenter;
			double temp = repel * (1.0 / distSurf - 1.0 / EdgeDis);
			repX += temp * nx;
			repY += temp * ny;
			if (distSurf < minSurfDist) minSurfDist = distSurf;
		}
	}

	// 合力
	double totalX = attrX + repX;
	double totalY = attrY + repY;

	// 雷达触发避障优先策略：在前方 +/-30deg 扇区内检测最小距离
	bool obstacleAhead = false;
	double minRange = 1e9;
	double sideSum = 0.0;

	if (LineNum > 0 && (int)LaserDist.size() == LineNum)
	{
		double TWO_PI = 2.0 * 3.14159265358979323846;
		double dAng = TWO_PI / (double)LineNum;
		double halfAngle = 30.0 * 3.14159265358979323846 / 180.0;
		int halfBins = max(1, (int)(halfAngle / dAng));
		int center = 0; // SimulateLaser: i=0 对应正前方
		double safetyDist = EdgeDis * 1.5;
		for (int k = -halfBins; k <= halfBins; ++k)
		{
			int idx = (center + k + LineNum) % LineNum;
			double r = LaserDist[idx];
			if (r <= 0) continue;
			if (r < minRange) minRange = r;
			if (r < safetyDist)
			{
				obstacleAhead = true;
				double relAng = k * dAng; // 相对机器人朝向
				double w = (safetyDist - r) / safetyDist;
				sideSum += w * sin(relAng);
			}
		}
	}

	if (obstacleAhead)
	{
		// 根据侧向权重决定避让方向：sideSum>0 表示更多障碍在左侧，应向右避让
		double turnAng = 60.0 * 3.14159265358979323846 / 180.0; // 60deg
		double desiredHeading = (sideSum > 0.0) ? (RobotTheta - turnAng) : (RobotTheta + turnAng);
		// 设置期望速度向量，交给 PID 产生具体控制
		ExpectVX = cos(desiredHeading) * 0.6; // 目标速度 0.6 m/s
		ExpectVY = sin(desiredHeading) * 0.6;
		TRACE(_T("ObsAvoid(LIDAR): minR=%.2f sideSum=%.2f desired=%.2f\n"), minRange, sideSum, desiredHeading);
	}
	else
	{
		// 无前方障碍，使用原有合力期望
		ExpectVX = GradientStep * totalX;
		ExpectVY = GradientStep * totalY;
		if (minSurfDist > 1e8) minSurfDist = -1.0;
		TRACE(_T("minDistSurf=%.2f distGoal=%.2f | attrX=%.2f repX=%.2f ExpectVX=%.2f ExpectVY=%.2f\n"),
			minSurfDist, distGoal, attrX, repX, ExpectVX, ExpectVY);
	}
}

//PID
void CAutoRobotDoc::PID()
{
	double expectV = sqrt(ExpectVX*ExpectVX + ExpectVY*ExpectVY);

	//�յ��ж�
	double dxEnd = FinX - RobotX;
	double dyEnd = FinY - RobotY;
	double distEnd = sqrt(dxEnd*dxEnd + dyEnd*dyEnd);
	if(distEnd < 0.6)
	{
		resV = 0;
		resW = 0;
		TotalDV = 0;
		LastDV = 0;
		return;
	}

	double desTheta = atan2(ExpectVY, ExpectVX);
	double errAng = desTheta - RobotTheta;
	while(errAng > 3.1415926) errAng -= 2*3.1415926;
	while(errAng < -3.1415926) errAng += 2*3.1415926;


	dV = expectV - RobotV;
	TotalDV += dV;

	if(TotalDV>8) TotalDV=8;
	if(TotalDV<-8) TotalDV=-8;

	resV = kp*dV + ki*TotalDV + kd*(dV-LastDV);
	LastDV = dV;

	if(resV > 3.0) resV = 3.0;
	if(resV < 0.0) resV = 0.0;

	resW = 0.7 * errAng;
	/*if (expectV < 0.05)
	{
		resW = 0.0;
	}*/
}

//��չ켣
void CAutoRobotDoc::ClearTag()
{
	TagNum = 0;
	ReTag = 0;
	memset(TagX, 0, sizeof(TagX));
	memset(TagY, 0, sizeof(TagY));
}

//�������˶�״̬
void CAutoRobotDoc::RobotSta(double dt)
{
	RobotX = RobotX + resV * cos(RobotTheta) * dt;
	RobotY = RobotY + resV * sin(RobotTheta) * dt;
	RobotTheta = RobotTheta + resW * dt;

	//���������»�������ʵ�ٶ�
	RobotV = resV;
	RobotW = resW;
}

//�����
double CAutoRobotDoc::Random(double m, double d)
{
	static bool hasSpare = false;
	static double spare;
	if (hasSpare)
	{
		hasSpare = false;
		return m + d * spare;
	}
	hasSpare = true;
	double u, v, s;
	do
	{
		u = (rand() / (double)RAND_MAX) * 2.0 - 1.0;
		v = (rand() / (double)RAND_MAX) * 2.0 - 1.0;
		s = u * u + v * v;
	} while (s >= 1.0 || s == 0);
	s = sqrt(-2.0 * log(s) / s);
	spare = v * s;
	return m + d * u * s;
}

//����
void CAutoRobotDoc::SaveSimData(CString filePath)
{
	ofstream fout(filePath.GetBuffer());
	fout << RobotX <<" "<<RobotY<<endl;
	fout << StartX <<" "<<StartY<<" "<<FinX<<" "<<FinY<<endl;
	fout << ObsX<<" "<<ObsY<<" "<<ObsR<<endl;
	fout << TagNum <<endl;
	for(int i=0;i<TagNum;i++)
	{
		fout << TagX[i] <<" "<<TagY[i]<<endl;
	}
	fout.close();
}

//�ط�
void CAutoRobotDoc::LoadPlayBack(CString filePath)
{
	ifstream fin(filePath.GetBuffer());
	fin >> RobotX >> RobotY;
	fin >> StartX >> StartY >> FinX >> FinY;
	fin >> ObsX >> ObsY >> ObsR;
	fin >> TagNum;
	for(int i=0;i<TagNum;i++)
	{
		fin >> TagX[i] >> TagY[i];
	}
	fin.close();
	IsPlayBack = true;
	IfRun = true;
}
//ʾ������ʾ���
void CAutoRobotDoc::CalcError()
{
	TRACE(_T("DataIndex = %d\n"), DataIndex);
	if(DataIndex >= DATA_BUFFER_LEN)
		DataIndex =0;

	double errG = sqrt((GPSX-RobotX)*(GPSX-RobotX)+(GPSY-RobotY)*(GPSY-RobotY));
	double errE = sqrt((ekfX-RobotX)*(ekfX-RobotX)+(ekfY-RobotY)*(ekfY-RobotY));
	// INS 与真实位置的误差，也记录到缓冲区用于示波器显示
	double errI = sqrt((INSX - RobotX) * (INSX - RobotX) + (INSY - RobotY) * (INSY - RobotY));
	//====�����޷���������֮ǰ�ض����ֵ====
	const double MAX_ERR = 1.2;
	if (errG > MAX_ERR) errG = MAX_ERR;
	if (errE > MAX_ERR) errE = MAX_ERR;
	if (errI > MAX_ERR) errI = MAX_ERR;
	ErrGps[DataIndex]=errG;
	ErrEKF[DataIndex]=errE;
	ErrIns[DataIndex] = errI;
	PidVData[DataIndex]=resV;
	DataIndex++;
}
// CAutoRobotDoc ���л�

void CAutoRobotDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: �ڴ���Ӵ洢����
	}
	else
	{
		// TODO: �ڴ���Ӽ��ش���
	}
}

#ifdef SHARED_HANDLERS

// ����ͼ��֧��
void CAutoRobotDoc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// �޸Ĵ˴����Ի����ĵ�����
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// ������������֧��
void CAutoRobotDoc::InitializeSearchContent()
{
	CString strSearchContent;
	// ���ĵ����������������ݡ�
	// ���ݲ���Ӧ�ɡ�;���ָ�

	// ����:  strSearchContent = _T("point;rectangle;circle;ole object;");
	SetSearchContent(strSearchContent);
}

void CAutoRobotDoc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CAutoRobotDoc ���

#ifdef _DEBUG
void CAutoRobotDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CAutoRobotDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CAutoRobotDoc 结束
void CAutoRobotDoc::AddRecord()
{
	SimRecord r;
	r.time = m_simTime;
	r.robotX = RobotX;
	r.robotY = RobotY;
	r.robotTheta = RobotTheta;

	r.gpsX = GPSX;
	r.gpsY = GPSY;
	r.insX = INSX;
	r.insY = INSY;

	r.ekfX = ekfX;
	r.ekfY = ekfY;
	r.ekfTheta = ekfTheta;

	r.resV = resV;
	r.resW = resW;

	//计算当前误差
	r.errGps = sqrt((GPSX - RobotX) * (GPSX - RobotX) + (GPSY - RobotY) * (GPSY - RobotY));
	r.errEKF = sqrt((ekfX - RobotX) * (ekfX - RobotX) + (ekfY - RobotY) * (ekfY - RobotY));

	m_records.push_back(r);
}
BOOL CAutoRobotDoc::SaveToCSV(CString filePath)
{
	if (m_records.empty())
		return FALSE;

	std::ofstream fout(filePath.GetBuffer());
	if (!fout.is_open())
		return FALSE;

	//表头
	fout << "time,robotX,robotY,robotTheta,gpsX,gpsY,insX,insY,ekfX,ekfY,ekfTheta,resV,resW,errGps,errEKF\n";
	for (auto& r : m_records)
	{
		fout
			<< r.time << ","
			<< r.robotX << "," << r.robotY << "," << r.robotTheta << ","
			<< r.gpsX << "," << r.gpsY << ","
			<< r.insX << "," << r.insY << ","
			<< r.ekfX << "," << r.ekfY << "," << r.ekfTheta << ","
			<< r.resV << "," << r.resW << ","
			<< r.errGps << "," << r.errEKF << "\n";
	}
	fout.close();
	return TRUE;
}
BOOL CAutoRobotDoc::LoadFromCSV(CString filePath)
{
	std::ifstream fin(filePath.GetBuffer());
	if (!fin.is_open())
		return FALSE;

	m_records.clear();
	std::string line;
	//跳过表头
	std::getline(fin, line);

	while (std::getline(fin, line))
	{
		SimRecord r;
		double t, rx, ry, rth, gx, gy, ix, iy, ex, ey, eth, v, w, eg, ee;
		//csv逗号分割
		if (sscanf(line.c_str(),
			"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
			&t, &rx, &ry, &rth, &gx, &gy, &ix, &iy, &ex, &ey, &eth, &v, &w, &eg, &ee) == 15)
		{
			r.time = t;
			r.robotX = rx; r.robotY = ry; r.robotTheta = rth;
			r.gpsX = gx; r.gpsY = gy;
			r.insX = ix; r.insY = iy;
			r.ekfX = ex; r.ekfY = ey; r.ekfTheta = eth;
			r.resV = v; r.resW = w;
			r.errGps = eg; r.errEKF = ee;
			m_records.push_back(r);
		}
	}
	fin.close();
	if (m_records.empty())
		return FALSE;

	//进入回放模式标记
	m_isReplay = true;
	m_replayIdx = 0;
	m_simTime = 0.0;
	IfRun = true;

	//回放时清空旧轨迹、示波器缓存
	ClearTag();
	DataIndex = 0;
	memset(ErrGps, 0, sizeof(ErrGps));
	memset(ErrEKF, 0, sizeof(ErrEKF));
	memset(PidVData, 0, sizeof(PidVData));

	return TRUE;
}
void CAutoRobotDoc::ApplyReplayStep()
{
	if (m_replayIdx >= (int)m_records.size())
	{
		//回放结束
		m_isReplay = false;
		IfRun = false;
		return;
	}

	SimRecord& r = m_records[m_replayIdx];

	//把记录赋值给仿真全部状态变量
	m_simTime = r.time;
	RobotX = r.robotX;
	RobotY = r.robotY;
	RobotTheta = r.robotTheta;

	GPSX = r.gpsX;
	GPSY = r.gpsY;
	INSX = r.insX;
	INSY = r.insY;

	ekfX = r.ekfX;
	ekfY = r.ekfY;
	ekfTheta = r.ekfTheta;

	resV = r.resV;
	resW = r.resW;
	RobotV = r.resV;
	RobotW = r.resW;

	//回放填充示波器缓冲区
	if (DataIndex < DATA_BUFFER_LEN)
	{
		ErrGps[DataIndex] = r.errGps;
		ErrEKF[DataIndex] = r.errEKF;
		PidVData[DataIndex] = r.resV;
		DataIndex++;
	}

	//回放绘制轨迹
	if (TagNum < 2000)
	{
		TagX[TagNum] = RobotX;
		TagY[TagNum] = RobotY;
		TagNum++;
	}

	m_replayIdx++;
}

