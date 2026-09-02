#pragma once

class CAutoRobotDoc;
// CParamDlg 对话框

class CParamDlg : public CDialog
{
	DECLARE_DYNAMIC(CParamDlg)

public:
	CParamDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CParamDlg();

// 对话框数据
	enum { IDD = IDD_DIALOG_PARAM };

	//添加指针成员
	CAutoRobotDoc * m_pDoc;//声明Doc指针

	//====和CAutoRobotDoc成员一一对应====
	double m_GPSFreq;		//GPS频率 1‑20Hz
	double m_GPSNoise;		//GPS噪声 0‑10m
	double m_INSFreq;		//INS频率10‑500Hz
	double m_INSNoise;		//INS速度噪声0‑1

	int m_LineNum;			//雷达点数36‑1440
	double m_MaxDis;		//量程
	double m_NoiseProb;		//噪点概率0‑1

	int m_NaviMode;
	int m_FusionMode;
	int m_AvoidMode;

	double m_GPSWeight;
	double m_Q00;
	double m_R00;

	double m_attract;		//引力
	double m_repel;			//斥力
	double m_EdgeDis;		//障碍物边界距离
	double m_GradientStep;	//梯度下降步长

	double m_kp;
	double m_ki;
	double m_kd;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	virtual void OnOK();

};
