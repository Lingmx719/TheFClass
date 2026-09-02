
// AutoRobotView.cpp : CAutoRobotView 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。

#ifndef SHARED_HANDLERS
#include "AutoRobot.h"
#endif

#include "AutoRobotDoc.h"
#include "AutoRobotView.h"

#include "ParamDlg.h"
#include <cmath>
#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CAutoRobotView

IMPLEMENT_DYNCREATE(CAutoRobotView, CView)

BEGIN_MESSAGE_MAP(CAutoRobotView, CView)
	// 标准打印命令
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CAutoRobotView::OnFilePrintPreview)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_COMMAND(ID_MENU_REACT, &CAutoRobotView::OnMenuReact)
	ON_COMMAND(ID_MENU_SAVE, &CAutoRobotView::OnMenuSave)
	ON_COMMAND(ID_MENU_SETBLOCK, &CAutoRobotView::OnMenuSetblock)
	ON_COMMAND(ID_MENU_SETFIN, &CAutoRobotView::OnMenuSetfin)
	ON_COMMAND(ID_MENU_SETSTART, &CAutoRobotView::OnMenuSetstart)
	ON_COMMAND(ID_MENU_START, &CAutoRobotView::OnMenuStart)
	ON_WM_LBUTTONDOWN()
	ON_WM_TIMER()
	ON_COMMAND(ID_MENU_ROBOT_PANEL, &CAutoRobotView::OnMenuRobotPanel)
	ON_COMMAND(ID_FILE_NEW, &CAutoRobotView::OnFileNew) //新增这一行

END_MESSAGE_MAP()

// CAutoRobotView 构造/析构

CAutoRobotView::CAutoRobotView()
{
	// TODO: 在此处添加构造代码

}

CAutoRobotView::~CAutoRobotView()
{
}

BOOL CAutoRobotView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CView::PreCreateWindow(cs);
}

void CAutoRobotView::DrawOscilloscope(CDC* pMemDC, CRect rcOsc, CAutoRobotDoc* pDoc)
{
	// 每次绘制示波器前，把示波器矩形填充白色
	pMemDC->FillSolidRect(&rcOsc, RGB(255, 255, 255));
	pMemDC->Rectangle(&rcOsc);
	int w = rcOsc.Width();
	int h = rcOsc.Height();
	int baseY = rcOsc.bottom;
	int bufLen = DATA_BUFFER_LEN;
	int validCnt = pDoc->DataIndex;
	if(validCnt<2) return;

	double scaleErr = 1.3;
	int startX = rcOsc.left;

	// 将示波器分为四条独立的水平带：
// 上：GPS (红)，其次 EKF (蓝)，其次 INS (橙)，底部 PID (绿)。每条带左侧写明标签。
// 使用不均等带高，扩大底部与中间用于更明显显示
	double r1 = 0.22; // GPS
	double r2 = 0.22; // EKF
	double r3 = 0.22; // INS
	double r4 = 1.0 - r1 - r2 - r3; // PID
	int gap = 2; // 带之间的小间隙
	int bandHTop = (int)floor(h * r1);
	int bandHMid = (int)floor(h * r2);
	int bandHIns = (int)floor(h * r3);
	int bandHBot = h - bandHTop - bandHMid - bandHIns; // 保证总高度一致
	CFont* pOldFont = NULL;
	// 准备字体用于标签
	LOGFONT lf;
	CFont* pGuiFont = CFont::FromHandle((HFONT)GetStockObject(DEFAULT_GUI_FONT));
	pGuiFont->GetLogFont(&lf);
	int minBandH = min(min(bandHTop, bandHMid), bandHBot);
	lf.lfHeight = max(12, minBandH / 4);
	CFont fontLabel; fontLabel.CreateFontIndirect(&lf);
	pOldFont = pMemDC->SelectObject(&fontLabel);

	// 绘制每一条带的曲线函数
	auto drawBand = [&](int bandIndex, const CString& label, CPen& pen, const double* data, double dataScale, double vCompress)
		{
			int offset = 0;
			int bandHLocal = bandHTop;
			if (bandIndex == 0) { offset = 0; bandHLocal = bandHTop; }
			else if (bandIndex == 1) { offset = bandHTop; bandHLocal = bandHMid; }
			else if (bandIndex == 2) { offset = bandHTop + bandHMid; bandHLocal = bandHIns; }
			else { offset = bandHTop + bandHMid + bandHIns; bandHLocal = bandHBot; }
			int top = rcOsc.top + offset + gap;
			int bottom = top + bandHLocal - 2 * gap;
			int bw = w - 40; // 留出左侧 40 像素用于标签
		int left = rcOsc.left + 40;
		int baseYb = top + (bottom - top);

		// 标签
		pMemDC->SetTextColor(RGB(0,0,0));
		pMemDC->TextOut(rcOsc.left + 4, top + 2, label);

		// 画边框区域
		CRect rband(left, top, rcOsc.right, bottom);
		pMemDC->DrawEdge(&rband, EDGE_SUNKEN, BF_RECT);

		// 画曲线
		CPen* pPrev = pMemDC->SelectObject(&pen);
		if (validCnt > 0)
		{
			int px0 = left + (int)((0.0 / bufLen) * bw);
			int py0 = baseYb - (int)((data[0] / dataScale) * (bottom - top) * vCompress);
			pMemDC->MoveTo(px0, py0);
			for (int i = 1; i < validCnt; ++i)
			{
				int px = left + (int)((double)i / bufLen * bw);
				int py = baseYb - (int)((data[i] / dataScale) * (bottom - top) * vCompress);
				// 允许一定超出但限制范围
				if (py < top - (bottom - top)) py = top - (bottom - top);
				if (py > bottom + (bottom - top)) py = bottom + (bottom - top);
				pMemDC->LineTo(px, py);
			}
		}
		pMemDC->SelectObject(pPrev);
	};

	// 数据指针准备（注意数组长度 DATA_BUFFER_LEN）
// 0: GPS (红)
	CPen penGps(PS_SOLID, 1, RGB(255, 0, 0));
	drawBand(0, _T("GPS"), penGps, pDoc->ErrGps, scaleErr, 1.2);
	// 1: EKF (蓝)
	CPen penEKF(PS_SOLID, 1, RGB(0, 0, 255));
	drawBand(1, _T("EKF"), penEKF, pDoc->ErrEKF, scaleErr, 1.2);
	// 2: INS (橙)
	CPen penIns(PS_SOLID, 1, RGB(255, 140, 0));
	drawBand(2, _T("INS"), penIns, pDoc->ErrIns, scaleErr, 1.2);
	// 3: PID (绿)
	CPen penPid(PS_SOLID, 1, RGB(0, 255, 0));
	drawBand(3, _T("PID 输出"), penPid, pDoc->PidVData, 3.0, 0.9);

	// 恢复字体
	pMemDC->SelectObject(pOldFont);
}



// CAutoRobotView 绘制

void CAutoRobotView::OnDraw(CDC* pDC)
{
	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	CRect rcClient;
	GetClientRect(&rcClient);

	CDC memDC;
	CBitmap bmpMem;
	memDC.CreateCompatibleDC(pDC);
	bmpMem.CreateCompatibleBitmap(pDC, rcClient.Width(), rcClient.Height());
	memDC.SelectObject(&bmpMem);
	memDC.FillSolidRect(&rcClient,RGB(255,255,255));

	//起点 绿色方块
	CPoint ptSt = WorldToScreen(pDoc->StartX,pDoc->StartY);
	CBrush brSt(RGB(0,200,0));
	memDC.SelectObject(&brSt);
	memDC.Rectangle(ptSt.x-8,ptSt.y-8,ptSt.x+8,ptSt.y+8);

	//终点红色方块
	CPoint ptFi = WorldToScreen(pDoc->FinX,pDoc->FinY);
	CBrush brFi(RGB(255,0,0));
	memDC.SelectObject(&brFi);
	memDC.Rectangle(ptFi.x-8,ptFi.y-8,ptFi.x+8,ptFi.y+8);

	//障碍物灰色圆
	CPoint ptObs = WorldToScreen(pDoc->ObsX,pDoc->ObsY);
	int obsRpx = (int)(pDoc->ObsR * 12.0);
	CBrush brObs(RGB(120,120,120));
	memDC.SelectObject(&brObs);
	memDC.Ellipse(ptObs.x-obsRpx,ptObs.y-obsRpx,ptObs.x+obsRpx,ptObs.y+obsRpx);

	//绘制多个障碍物
	for (auto& ob : pDoc->m_obsList)
	{
		CPoint c = WorldToScreen(ob.x, ob.y);
		//求圆边上一点，x方向偏移一个半径，换算屏幕得到像素半径
		CPoint ptEdge = WorldToScreen(ob.x + ob.z, ob.y);
		int rPix = abs(ptEdge.x - c.x);

		// 使用 memDC 绘制到后续 BitBlt 的位图上，保证不会被覆盖
		CBrush brOb(RGB(120,120,120));
		CBrush* pOldBr = memDC.SelectObject(&brOb);
		CPen penOb(PS_SOLID, 1, RGB(80,80,80));
		CPen* pOldPen = memDC.SelectObject(&penOb);
		memDC.Ellipse(c.x - rPix, c.y - rPix, c.x + rPix, c.y + rPix);
		memDC.SelectObject(pOldPen);
		memDC.SelectObject(pOldBr);
	}

	// 绘制雷达光束
	int lines = pDoc->LineNum;
	int realLines = min(lines, (int)pDoc->LaserDist.size());
	double dAng = 2 * 3.1415926 / realLines;

	for (int i = 0; i < realLines; ++i)
	{
		double dist = pDoc->LaserDist[i];
		double ang = pDoc->RobotTheta + i * dAng;

		//激光终点世界坐标
		double wx = pDoc->RobotX + dist * cos(ang);
		double wy = pDoc->RobotY + dist * sin(ang);

		//世界坐标转屏幕点
		CPoint p0 = WorldToScreen(pDoc->RobotX, pDoc->RobotY);
		CPoint p1 = WorldToScreen(wx, wy);

		memDC.MoveTo(p0);
		memDC.LineTo(p1);
	}

	//机器人本体+航向箭头【改成三角形机器人】
	CPoint ptRob = WorldToScreen(pDoc->RobotX, pDoc->RobotY);
	CBrush brRob(RGB(0, 0, 0));
	memDC.SelectObject(&brRob);

	double theta = pDoc->RobotTheta;
	int robotSize = 14;   //三角形大小，可调节
	const double PI = 3.141592653589793;
	//三角形三个顶点
	CPoint triPts[3];
	// 顶点0：车头，朝向theta方向
	triPts[0].x = ptRob.x + (int)(robotSize * cos(theta));
	triPts[0].y = ptRob.y + (int)(robotSize * sin(theta));

	// 顶点1：车尾左后方 theta + 120度
	triPts[1].x = ptRob.x + (int)(robotSize * cos(theta + 2.0 * PI / 3.0));
	triPts[1].y = ptRob.y + (int)(robotSize * sin(theta + 2.0 * PI / 3.0));

	// 顶点2：车尾右后方 theta - 120度
	triPts[2].x = ptRob.x + (int)(robotSize * cos(theta - 2.0 * PI / 3.0));
	triPts[2].y = ptRob.y + (int)(robotSize * sin(theta - 2.0 * PI / 3.0));

	//绘制实心三角形
	memDC.Polygon(triPts, 3);

	//绘制运动轨迹
	memDC.SelectStockObject(PS_SOLID);
	for(int i=1;i < pDoc->TagNum;i++)
	{
		CPoint p0 = WorldToScreen(pDoc->TagX[i-1],pDoc->TagY[i-1]);
		CPoint p1 = WorldToScreen(pDoc->TagX[i],pDoc->TagY[i]);
		memDC.MoveTo(p0);
		memDC.LineTo(p1);
	}

	// 单GPS模式：只绘制GPS紫色点
	if (pDoc->NaviMode == 0)
	{
		CPoint ptGps = WorldToScreen(pDoc->GPSX, pDoc->GPSY);
		CPen penPurple(PS_SOLID, 1, RGB(160, 0, 220));
		CBrush brPurple(RGB(160, 0, 220));
		CPen* pOldPen = memDC.SelectObject(&penPurple);
		CBrush* pOldBr = memDC.SelectObject(&brPurple);
		memDC.Ellipse(ptGps.x - 4, ptGps.y - 4, ptGps.x + 4, ptGps.y + 4);
		memDC.SelectObject(pOldPen);
		memDC.SelectObject(pOldBr);
	}
	// 单INS模式：只绘制INS黄色点
	else if (pDoc->NaviMode == 1)
	{
		CPoint ptIns = WorldToScreen(pDoc->INSX, pDoc->INSY);
		CPen penYellow(PS_SOLID, 1, RGB(255, 210, 0));
		CBrush brYellow(RGB(255, 210, 0));
		CPen* pOldPen = memDC.SelectObject(&penYellow);
		CBrush* pOldBr = memDC.SelectObject(&brYellow);
		memDC.Ellipse(ptIns.x - 4, ptIns.y - 4, ptIns.x + 4, ptIns.y + 4);
		memDC.SelectObject(pOldPen);
		memDC.SelectObject(pOldBr);
	}
	// GPS‑INS融合模式：GPS、INS、融合点全部绘制
	else if (pDoc->NaviMode == 2)
	{
		//GPS紫色小圆
		CPoint ptGps = WorldToScreen(pDoc->GPSX, pDoc->GPSY);
		CPen penPurple(PS_SOLID, 1, RGB(160, 0, 220));
		CBrush brPurple(RGB(160, 0, 220));
		CPen* pOldPen = memDC.SelectObject(&penPurple);
		CBrush* pOldBr = memDC.SelectObject(&brPurple);
		memDC.Ellipse(ptGps.x - 4, ptGps.y - 4, ptGps.x + 4, ptGps.y + 4);
		memDC.SelectObject(pOldPen);
		memDC.SelectObject(pOldBr);

		//INS黄色小圆
		CPoint ptIns = WorldToScreen(pDoc->INSX, pDoc->INSY);
		CPen penYellow(PS_SOLID, 1, RGB(255, 210, 0));
		CBrush brYellow(RGB(255, 210, 0));
		pOldPen = memDC.SelectObject(&penYellow);
		pOldBr = memDC.SelectObject(&brYellow);
		memDC.Ellipse(ptIns.x - 4, ptIns.y - 4, ptIns.x + 4, ptIns.y + 4);
		memDC.SelectObject(pOldPen);
		memDC.SelectObject(pOldBr);

		//融合估计点
		CPoint fusionPt = WorldToScreen(pDoc->FusionX, pDoc->FusionY);
		if (pDoc->FusionMode == 0)
		{
			//加权平均 →红色小圆
			CBrush brRed(RGB(255, 0, 0));
			pOldBr = memDC.SelectObject(&brRed);
			memDC.Ellipse(fusionPt.x - 4, fusionPt.y - 4, fusionPt.x + 4, fusionPt.y + 4);
			memDC.SelectObject(pOldBr);
		}
		else if (pDoc->FusionMode == 1)
		{
			//EKF卡尔曼滤波 →蓝色小圆
			CBrush brEKF(RGB(0, 0, 255));
			pOldBr = memDC.SelectObject(&brEKF);
			memDC.Ellipse(fusionPt.x - 4, fusionPt.y - 4, fusionPt.x + 4, fusionPt.y + 4);
			memDC.SelectObject(pOldBr);
		}
	}


	//右下角示波器
// 将示波器区域略微放大，使曲线更易观察
	CRect rcOsc;
	// 右下角位置，宽度和高度相对于窗口略微扩大
	rcOsc.left = rcClient.Width() - 420; // 原来是 -340，增大宽度
	rcOsc.top = rcClient.Height() - 320;  // 原来是 -240，增大高度
	rcOsc.right = rcClient.Width() - 10;  // 微调右边距
	rcOsc.bottom = rcClient.Height() - 20;
	DrawOscilloscope(&memDC, rcOsc, pDoc);

	// 右上角实时激光雷达小视图（极坐标，显示障碍物和雷达点）
	{
		int size = 180;
		CRect rcL;
		rcL.right = rcClient.Width() - 10;
		rcL.left = rcL.right - size;
		rcL.top = 10;
		rcL.bottom = rcL.top + size;
		// 背景与边框
		memDC.FillSolidRect(&rcL, RGB(245, 255, 245));
		memDC.DrawEdge(&rcL, BDR_RAISEDINNER, BF_RECT);

		// 中心与半径（像素）
		int cx = (rcL.left + rcL.right) / 2;
		int cy = (rcL.top + rcL.bottom) / 2;
		int rPix = min((rcL.Width()-16)/2, (rcL.Height()-16)/2);

		// 画最大量程圆和中间刻度
		CPen penRange(PS_DOT, 1, RGB(180,180,180));
		CPen* pOldPen = memDC.SelectObject(&penRange);
		for (int k=1;k<=3;k++) {
			int rr = rPix * k / 3;
			memDC.Ellipse(cx-rr, cy-rr, cx+rr, cy+rr);
		}
		memDC.SelectObject(pOldPen);

		// 画机器人朝向（小箭头）
		CPen penRobot(PS_SOLID, 2, RGB(0,0,0));
		pOldPen = memDC.SelectObject(&penRobot);
		double theta = pDoc->RobotTheta;
		int hx = cx + (int)( (rPix*0.5) * cos(theta) );
		int hy = cy + (int)( (rPix*0.5) * sin(theta) );
		memDC.MoveTo(cx, cy);
		memDC.LineTo(hx, hy);
		memDC.SelectObject(pOldPen);

		// 绘制雷达点（小圆），红色表示近距离点
		int lines = max(1, pDoc->LineNum);
		double dAng = 2 * 3.1415926 / lines;
		for (int i=0;i<lines;i++) {
			double range = pDoc->LaserDist[i];
			if (range <= 0 || range > pDoc->MaxDis*1.2) continue;
			double ang = i * dAng; // 相对于 RobotTheta
			double relx = (range / pDoc->MaxDis) * rPix * cos(ang + theta);
			double rely = (range / pDoc->MaxDis) * rPix * sin(ang + theta);
			int px = cx + (int)relx;
			int py = cy + (int)rely;
			// 颜色：非常近的点为红，远的为橙/黄
			int colv = (int)(255.0 * (1.0 - min(range / pDoc->MaxDis, 1.0)));
			COLORREF c = RGB(255, colv/2, 0);
			CBrush br(c);
			CBrush* pOldBr = memDC.SelectObject(&br);
			memDC.Ellipse(px-2, py-2, px+2, py+2);
			memDC.SelectObject(pOldBr);
		}

		// 绘制已知障碍物（来自 m_obsList），绿色圆圈
		for (auto &ob : pDoc->m_obsList) {
			double dx = ob.x - pDoc->RobotX;
			double dy = ob.y - pDoc->RobotY;
			double dist = sqrt(dx*dx + dy*dy);
			if (dist > pDoc->MaxDis*1.2) continue;
			double ang = atan2(dy, dx);
			double relx = (dist / pDoc->MaxDis) * rPix * cos(ang - theta);
			double rely = (dist / pDoc->MaxDis) * rPix * sin(ang - theta);
			int px = cx + (int)relx;
			int py = cy + (int)rely;
			int rr = max(2, (int)(ob.z / pDoc->MaxDis * rPix));
			CBrush brOb(RGB(0,160,0));
			CBrush* pOldBr = memDC.SelectObject(&brOb);
			memDC.Ellipse(px-rr, py-rr, px+rr, py+rr);
			memDC.SelectObject(pOldBr);
		}

		// 提示文字（量程）
		CString s; s.Format(_T("LIDAR range: %.1f m"), pDoc->MaxDis);
		memDC.TextOut(rcL.left+6, rcL.bottom-18, s);
	}

	//贴到屏幕
	pDC->BitBlt(0,0,rcClient.Width(),rcClient.Height(),&memDC,0,0,SRCCOPY);


	// 在屏幕左下角绘制实时导航数据：GPS, INS, 融合结果
	{
		const int margin = 8;
		const int boxW = 260;
		const int boxH = 70;
		CRect rcBox(rcClient.left + margin, rcClient.bottom - boxH - margin, rcClient.left + margin + boxW, rcClient.bottom - margin);
		// 半透明背景不可直接使用，这里用浅色填充
		pDC->FillSolidRect(&rcBox, RGB(250, 250, 240));
		pDC->DrawEdge(&rcBox, BDR_SUNKENOUTER, BF_RECT);

		CString s1, s2, s3;
		// GPS
		s1.Format(_T("GPS:  X=%.2f  Y=%.2f  (w=%.2f)"), pDoc->GPSX, pDoc->GPSY, pDoc->GPSWeight);
		// INS
		s2.Format(_T("INS:  X=%.2f  Y=%.2f  (w=%.2f)"), pDoc->INSX, pDoc->INSY, pDoc->INSWeight);
		// Fusion
		s3.Format(_T("FUS:  X=%.2f  Y=%.2f  mode=%d"), pDoc->FusionX, pDoc->FusionY, pDoc->FusionMode);

		CFont* pGuiFontLocal = CFont::FromHandle((HFONT)GetStockObject(DEFAULT_GUI_FONT));
		CFont* pOldF = pDC->SelectObject(pGuiFontLocal);
		COLORREF oldCol = pDC->SetTextColor(RGB(0,0,0));
		pDC->TextOut(rcBox.left + 6, rcBox.top + 6, s1);
		pDC->TextOut(rcBox.left + 6, rcBox.top + 26, s2);
		pDC->TextOut(rcBox.left + 6, rcBox.top + 46, s3);
		pDC->SetTextColor(oldCol);
		pDC->SelectObject(pOldF);
	}
}


// CAutoRobotView 打印


void CAutoRobotView::OnFilePrintPreview()
{
#ifndef SHARED_HANDLERS
	AFXPrintPreview(this);
#endif
}

BOOL CAutoRobotView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 默认准备
	return DoPreparePrinting(pInfo);
}

void CAutoRobotView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加额外的打印前进行的初始化过程
}

void CAutoRobotView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加打印后进行的清理过程
}

void CAutoRobotView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
	OnContextMenu(this, point);
}

void CAutoRobotView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CAutoRobotView 诊断

#ifdef _DEBUG
void CAutoRobotView::AssertValid() const
{
	CView::AssertValid();
}

void CAutoRobotView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CAutoRobotDoc* CAutoRobotView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CAutoRobotDoc)));
	return (CAutoRobotDoc*)m_pDocument;
}
#endif //_DEBUG


// CAutoRobotView 消息处理程序


void CAutoRobotView::OnMenuReact()
{
	// TODO: 
	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	CFileDialog dlg(TRUE, _T("csv"), _T("robot_sim.csv"),
		OFN_HIDEREADONLY, _T("CSV文件(*.csv)|*.csv||"));
	if (dlg.DoModal() == IDOK)
	{
		BOOL ok = pDoc->LoadFromCSV(dlg.GetPathName());
		if (ok)
		{
			SetTimer(1, 50, NULL);
			AfxMessageBox(_T("CSV回放加载成功，开始回放"));
		}
		else
		{
			AfxMessageBox(_T("CSV文件读取失败！"));
		}
	}
}


void CAutoRobotView::OnMenuSave()//保存仿真轨迹txt
{
	// TODO: 在此添加命令处理程序代码

	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	CFileDialog dlg(FALSE, _T("csv"), _T("robot_sim.csv"),
		OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, _T("CSV文件(*.csv)|*.csv||"));
	if (dlg.DoModal() == IDOK)
	{
		BOOL ok = pDoc->SaveToCSV(dlg.GetPathName());
		if (ok)
		{
			AfxMessageBox(_T("CSV保存完成！"));
		}
		else
		{
			AfxMessageBox(_T("无仿真记录，保存失败，请先运行仿真！"));
		}
	}
}


void CAutoRobotView::OnMenuSetblock()
{
	// TODO: 在此添加命令处理程序代码M
	CAutoRobotDoc* pDoc = GetDocument();
    ASSERT_VALID(pDoc);
    pDoc->DrawMode = 3;
}


void CAutoRobotView::OnMenuSetfin()
{
	// TODO: 在此添加命令处理程序代码
	CAutoRobotDoc* pDoc = GetDocument();
    ASSERT_VALID(pDoc);
    pDoc->DrawMode = 2;
}


void CAutoRobotView::OnMenuSetstart()
{
	// TODO: 在此添加命令处理程序代码
	CAutoRobotDoc* pDoc = GetDocument();
    ASSERT_VALID(pDoc);
    pDoc->DrawMode = 1;
}


void CAutoRobotView::OnMenuStart()
{
	// TODO: 在此添加命令处理程序代码
	CAutoRobotDoc* pDoc = GetDocument();
    ASSERT_VALID(pDoc);

	if (pDoc->IfRun == false)
	{
		//点击开始，退出回放模式
		pDoc->m_isReplay = false;
		pDoc->m_replayIdx = 0;
		pDoc->m_simTime = 0.0;
		pDoc->m_records.clear();

		pDoc->RobotX = pDoc->StartX;
		pDoc->RobotY = pDoc->StartY;
		pDoc->RobotTheta = 0;

		pDoc->ClearTag();	//清空轨迹
		pDoc->DataIndex = 0; //重置示波器缓冲区索引
		memset(pDoc->ErrGps, 0, sizeof(pDoc->ErrGps));
		memset(pDoc->ErrEKF, 0, sizeof(pDoc->ErrEKF));
		memset(pDoc->PidVData, 0, sizeof(pDoc->PidVData));

		pDoc->IfRun = true;
		SetTimer(1, 50, NULL);
	}
	else
	{
		//再点一次开始停止
		pDoc->IfRun = false;
		pDoc->m_isReplay = false;
		KillTimer(1);
	}
	Invalidate(FALSE);
}

//坐标转换
CPoint CAutoRobotView::WorldToScreen(double wx, double wy)
{
	const double scale = 12.0;
	const double offX = 60;
	const double offY = 60;
	int sx = (int)(offX + wx * scale);
	int sy = (int)(offY + wy * scale);
	return CPoint(sx, sy);
}

void CAutoRobotView::ScreenToWorld(CPoint pt, double& wx, double& wy)
{
	const double scale =12.0;
	const double offX =60;
	const double offY =60;
	wx = (pt.x - offX)/scale;
	wy = (pt.y - offY)/scale;
}




//鼠标左键设置起点终点障碍物
void CAutoRobotView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if(!pDoc)
	{
		CView::OnLButtonDown(nFlags, point);
		return;
	}
	double wx, wy;// 把屏幕鼠标像素坐标point，转换成仿真世界坐标 wx wy
	ScreenToWorld(point, wx, wy);

	if(pDoc->DrawMode ==1)// DrawMode ==1：模式1，设置起点
	{
		pDoc->StartX = wx; pDoc->StartY = wy;// 将转换后的世界坐标保存为机器人起点
	}
	else if(pDoc->DrawMode ==2)// DrawMode ==2：模式2，设置终点
	{
		pDoc->FinX = wx; pDoc->FinY = wy;//保存终点坐标
	}
	else if(pDoc->DrawMode ==3)// DrawMode ==3：模式3，设置障碍物
	{
		// 支持设置多个障碍物：把障碍物加入文档的 m_obsList
		CAutoRobotDoc::CObs ob;
		ob.x = wx; ob.y = wy; ob.z = 2.0; // 半径固定 2.0
		pDoc->m_obsList.push_back(ob);
		// 兼容原有单个障碍物变量
		pDoc->ObsX = wx; pDoc->ObsY = wy; pDoc->ObsR = 2.0;
	}
	pDoc->UpdateAllViews(NULL);



	CView::OnLButtonDown(nFlags, point);
}

void CAutoRobotDoc::SimulateLaser()//雷达
{
	int lines = min(LineNum, (int)LaserDist.max_size());//安全处理：lines取两者最小，防止越界
	LaserDist.assign(lines, MaxDis);
	const double PI = 3.14159265358793;
	double dAngle = 2 * PI / lines;
	double maxR = MaxDis;

	//LaserDist.assign(lines, maxR);

	for (int i = 0; i < lines; i++)
	{
		double ang = RobotTheta + i * dAngle;
		double dx = cos(ang);
		double dy = sin(ang);
		double minDist = maxR;

		//①扫描多障碍物列表 m_obsList
		for (auto& ob : m_obsList)
		{
			double ox = ob.x;
			double oy = ob.y;
			double r = ob.z;

			double fx = RobotX - ox;
			double fy = RobotY - oy;

			double a = dx * dx + dy * dy;
			double b = 2 * (fx * dx + fy * dy);
			double c = fx * fx + fy * fy - r * r;

			double delta = b * b - 4 * a * c;
			if (delta >= 0)
			{
				double t1 = (-b - sqrt(delta)) / (2 * a);
				if (t1 > 1e-6 && t1 < minDist)
				{
					minDist = t1;
				}
			}
		}

		//②【新增】兼容原来旧的单个障碍物 ObsX ObsY ObsR，也参与激光检测
		{
			double ox = ObsX;
			double oy = ObsY;
			double r = ObsR;

			double fx = RobotX - ox;
			double fy = RobotY - oy;

			double a = dx * dx + dy * dy;
			double b = 2 * (fx * dx + fy * dy);
			double c = fx * fx + fy * fy - r * r;

			double delta = b * b - 4 * a * c;
			if (delta >= 0)
			{
				double t1 = (-b - sqrt(delta)) / (2 * a);
				if (t1 > 1e-6 && t1 < minDist)
				{
					minDist = t1;
				}
			}
		}

		//模拟雷达噪点
		double rnd = (double)rand() / RAND_MAX;
		if (rnd < NoiseProb)
		{
			minDist = minDist * ((double)rand() / RAND_MAX * 0.4 + 0.3);
		}
		LaserDist[i] = minDist;
	}
}

void CAutoRobotView::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);

	if (nIDEvent == 1)
	{
		if (pDoc->IfRun)
		{
			if (pDoc->m_isReplay)
			{
				//回放模式：执行回放一帧，不跑SimStep
				pDoc->ApplyReplayStep();
			}
			else
			{
				//正常仿真
				pDoc->SimStep();
			}
			Invalidate(FALSE);	//触发OnDraw重绘
		}
	}
	CView::OnTimer(nIDEvent);
}




//1 参数设置对话框，弹出模态对话框修改噪声、Q R 、kp ki kd、雷达参数
void CAutoRobotView::OnMenuRobotPanel()
{
	// TODO: 在此添加命令处理程序代码
	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);

	CParamDlg dlg;////对话框类 CParamDlg，对应 ParamDlg.h
	dlg.m_pDoc = pDoc;
	//把Doc当前参数传入对话框
	dlg.m_GPSFreq = pDoc->GPSFreq;
	dlg.m_GPSNoise = pDoc->GPSNoise;
	dlg.m_INSFreq = pDoc->INSFreq;
	dlg.m_INSNoise = pDoc->INSNoise;
	dlg.m_LineNum   = pDoc->LineNum;
	dlg.m_MaxDis    = pDoc->MaxDis;
	dlg.m_NoiseProb = pDoc->NoiseProb;

	dlg.m_NaviMode  = pDoc->NaviMode;
	dlg.m_FusionMode= pDoc->FusionMode;
	dlg.m_AvoidMode = pDoc->AvoidMode;

	dlg.m_GPSWeight = pDoc->GPSWeight;
	dlg.m_Q00       = pDoc->Q[0][0];
	dlg.m_R00       = pDoc->R[0][0];

	dlg.m_attract   = pDoc->attract;
	dlg.m_repel     = pDoc->repel;
	dlg.m_EdgeDis   = pDoc->EdgeDis;
	dlg.m_GradientStep = pDoc->GradientStep;

	dlg.m_kp = pDoc->kp;
	dlg.m_ki = pDoc->ki;
	dlg.m_kd = pDoc->kd;

	if(dlg.DoModal() == IDOK)
	{
		// 用户点确定，对话框校验已经在CParamDlg::OnOK完成
		// 将对话框的临时变量写回Doc（真正修改仿真参数）
		CAutoRobotDoc* pDoc = GetDocument();
		pDoc->GPSFreq = dlg.m_GPSFreq;
		pDoc->GPSNoise = dlg.m_GPSNoise;
		pDoc->INSFreq = dlg.m_INSFreq;
		pDoc->INSNoise = dlg.m_INSNoise;
		pDoc->LineNum  = dlg.m_LineNum;
		pDoc->MaxDis   = dlg.m_MaxDis;
		pDoc->NoiseProb= dlg.m_NoiseProb;

		pDoc->NaviMode  = dlg.m_NaviMode;
		pDoc->FusionMode= dlg.m_FusionMode;
		pDoc->AvoidMode = dlg.m_AvoidMode;

		pDoc->GPSWeight = dlg.m_GPSWeight;
		pDoc->Q[0][0]   = dlg.m_Q00;
		pDoc->R[0][0]   = dlg.m_R00;

		pDoc->attract   = dlg.m_attract;
		pDoc->repel     = dlg.m_repel;
		pDoc->EdgeDis   = dlg.m_EdgeDis;
		pDoc->GradientStep = dlg.m_GradientStep;

		pDoc->kp = dlg.m_kp;
		pDoc->ki = dlg.m_ki;
		pDoc->kd = dlg.m_kd;
		pDoc->UpdateAllViews(NULL);
		//刷新视图，界面立刻更新
		Invalidate();
	}



}
void CAutoRobotView::OnFileNew()
{
	// 点文件新建，先把定时器杀死！！
	KillTimer(1);

	// 获取文档指针，调用文档的重置函数
	CAutoRobotDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);

	// 文档自己做初始化重置，清空机器人、轨迹、仿真状态
	pDoc->OnNewDocument();

	Invalidate(); //刷新画面
}