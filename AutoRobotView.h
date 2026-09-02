
// AutoRobotView.h : CAutoRobotView 类的接口
//

#pragma once


class CAutoRobotView : public CView
{
protected: // 仅从序列化创建
	CAutoRobotView();
	DECLARE_DYNCREATE(CAutoRobotView)

// 特性
public:
	CAutoRobotDoc* GetDocument() const;

// 操作
public:

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// 实现
public:
	virtual ~CAutoRobotView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnMenuReact();
	afx_msg void OnMenuSave();
	afx_msg void OnMenuSetblock();
	afx_msg void OnMenuSetfin();
	afx_msg void OnMenuSetstart();
	afx_msg void OnMenuStart();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnMenuRobotPanel();//机器人面板菜单响应
	afx_msg void OnFileNew();

	private:
	CPoint WorldToScreen(double wx, double wy);
	void ScreenToWorld(CPoint pt, double& wx, double& wy);
	void DrawOscilloscope(CDC* pMemDC, CRect rcOsc, CAutoRobotDoc* pDoc);
};

#ifndef _DEBUG  // AutoRobotView.cpp 中的调试版本
inline CAutoRobotDoc* CAutoRobotView::GetDocument() const
   { return reinterpret_cast<CAutoRobotDoc*>(m_pDocument); }
#endif

