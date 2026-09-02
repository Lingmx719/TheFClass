// ParamDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "AutoRobot.h"
#include "ParamDlg.h"
#include "afxdialogex.h"
#include "AutoRobotDoc.h"

// CParamDlg 对话框

IMPLEMENT_DYNAMIC(CParamDlg, CDialog)

CParamDlg::CParamDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CParamDlg::IDD, pParent)
{
	//m_pDoc = nullptr;//初始化指针
	m_GPSFreq = 10;
	m_GPSNoise = 0;
	m_INSFreq = 100;
	m_INSNoise = 0;

	m_LineNum = 360;
	m_MaxDis = 0;
	m_NoiseProb = 0;

	m_NaviMode = 2;
	m_FusionMode = 0;
	m_AvoidMode = 0;

	m_GPSWeight = 0;
	m_Q00 = 0;
	m_R00 = 0;

	m_attract = 0;
	m_repel = 0;
	m_EdgeDis = 0;
	m_GradientStep = 0;

	m_kp = 0;
	m_ki = 0;
	m_kd = 0;
}

CParamDlg::~CParamDlg()
{
}

void CParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);

	DDX_Text(pDX, IDC_EDIT_GPSFREQ, m_GPSFreq);
	DDX_Text(pDX, IDC_EDIT_INSFREQ, m_INSFreq);

	DDX_Text(pDX, IDC_EDIT_GPSNOISE, m_GPSNoise);
	DDX_Text(pDX, IDC_EDIT_INSNOISE, m_INSNoise);
	DDX_Text(pDX, IDC_EDIT_LINENUM, m_LineNum);
	DDX_Text(pDX, IDC_EDIT_MAXDIS, m_MaxDis);
	DDX_Text(pDX, IDC_EDIT_NOISEPROB, m_NoiseProb);

	DDX_Radio(pDX, IDC_RADIO_GPS, m_NaviMode);
	DDX_CBIndex(pDX, IDC_COMBO_FUSION, m_FusionMode);
	DDX_Text(pDX, IDC_EDIT_GPS_WEIGHT, m_GPSWeight);
	DDX_Text(pDX, IDC_EDIT_Q00, m_Q00);
	DDX_Text(pDX, IDC_EDIT_R00, m_R00);

	DDX_CBIndex(pDX, IDC_COMBO_AVOID, m_AvoidMode);
	DDX_Text(pDX, IDC_EDIT_ATTRACT, m_attract);
	DDX_Text(pDX, IDC_EDIT_REPEL, m_repel);
	DDX_Text(pDX, IDC_EDIT_EDGEDIS, m_EdgeDis);
	DDX_Text(pDX, IDC_EDIT_GRADSTEP, m_GradientStep);

	DDX_Text(pDX, IDC_EDIT_KP, m_kp);
	DDX_Text(pDX, IDC_EDIT_KI, m_ki);
	DDX_Text(pDX, IDC_EDIT_KD, m_kd);
}


BEGIN_MESSAGE_MAP(CParamDlg, CDialog)

END_MESSAGE_MAP()


// CParamDlg 消息处理程序


BOOL CParamDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  在此添加额外的初始化
	CComboBox* pFusion = (CComboBox*)GetDlgItem(IDC_COMBO_FUSION);//下拉框初始化
	pFusion->AddString(_T("加权平均"));
	pFusion->AddString(_T("EKF滤波"));
	pFusion->SetCurSel(0);//默认选择第0项
	CComboBox* pAvoid = (CComboBox*)GetDlgItem(IDC_COMBO_AVOID);
	pAvoid->AddString(_T("人工势场法"));
	pAvoid->SetCurSel(0);//默认选择第0项
	//m_NaviMode = m_pDoc->NaviMode;//把文档的NaviMode同步到对话框控件
	//UpdateData(FALSE); // FALSE：把成员变量的值刷新到界面控件

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}


void CParamDlg::OnOK()
{
	// TODO: 在此添加专用代码和/或调用基类
	if (!UpdateData(TRUE))
		return;
	//TRACE(_T("对话框读取：m_NaviMode=%d  m_FusionMode=%d\n"), m_NaviMode, m_FusionMode);
	//调试输出：看对话框有没有拿到你输入的数字
	TRACE(_T("[对话框内部] GPSNoise=%lf  kp=%lf\n"), m_GPSNoise, m_kp);

	//安全判断，防止指针为空再崩溃
	if (m_pDoc == nullptr)
	{
		CDialog::OnOK();
		return;
	}

	//简单数值合法性校验
	if (m_GPSFreq < 1 || m_GPSFreq>20)
	{
		AfxMessageBox(_T("GPS频率范围1‑20Hz"));
		return;
	}
	//GPS噪声(0‑10m)
	if (m_GPSNoise < 0 || m_GPSNoise>10)
	{
		AfxMessageBox(_T("GPS噪声范围0‑10m"));
		return;
	}
	//INS频率(10‑500Hz)
	if (m_INSFreq < 10 || m_INSFreq>500)
	{
		AfxMessageBox(_T("INS频率范围10‑500Hz"));
		return;
	}
	//INS速度噪声(0‑1)
	if (m_INSNoise < 0 || m_INSNoise>1)
	{
		AfxMessageBox(_T("INS速度噪声范围0‑1"));
		return;
	}
	if(m_LineNum <36 || m_LineNum>1440)
	{
		AfxMessageBox(_T("雷达点数范围36‑1440"));
		return;
	}
	if(m_NoiseProb <0 || m_NoiseProb>1)
	{
		AfxMessageBox(_T("噪点概率0‑1之间"));
		return;
	}
	if(m_kp<0 || m_ki<0 || m_kd<0)
	{
		AfxMessageBox(_T("PID参数不能负数"));
		return;
	}
	/*if (m_NaviMode < 0 || m_NaviMode>2)
	{
		m_NaviMode = 0; //异常强制切单GPS
	}
	/*if (IsDlgButtonChecked(IDC_RADIO_GPS))
	{
		m_NaviMode = 0;
	}
	else if (IsDlgButtonChecked(IDC_RADIO_INS))
	{
		m_NaviMode = 1;
	}
	else if (IsDlgButtonChecked(IDC_RADIO_FUSION))
	{
		m_NaviMode = 2;
	}*/
	//把对话框的值赋值给Doc
	m_pDoc->GPSFreq = m_GPSFreq;
	m_pDoc->GPSNoise = m_GPSNoise;
	m_pDoc->INSFreq = m_INSFreq;
	m_pDoc->INSNoise = m_INSNoise;

	m_pDoc->LineNum = m_LineNum;
	m_pDoc->LaserDist.resize(m_pDoc->LineNum);//避免vector越界崩溃
	m_pDoc->SimulateLaser();
	m_pDoc->MaxDis = m_MaxDis;
	m_pDoc->NoiseProb = m_NoiseProb;

	m_pDoc->NaviMode = m_NaviMode;//赋值给文档的导航模式
	m_pDoc->FusionMode = m_FusionMode;
	m_pDoc->AvoidMode = m_AvoidMode;

	m_pDoc->GPSWeight = m_GPSWeight;
	m_pDoc->Q[0][0] = m_Q00;
	m_pDoc->R[0][0] = m_R00;

	m_pDoc->attract = m_attract;
	m_pDoc->repel = m_repel;
	m_pDoc->EdgeDis = m_EdgeDis;
	m_pDoc->GradientStep = m_GradientStep;

	m_pDoc->kp = m_kp;
	m_pDoc->ki = m_ki;
	m_pDoc->kd = m_kd;

	m_pDoc->UpdateAllViews(NULL); //通知View，画面重新绘制OnDraw！！

	CDialog::OnOK();
}


