#pragma once

#include "presentation/qt/viewmodels/WorkerViewModel.hpp"
#include "presentation/qt/widgets/SlotGridWidget.hpp"  // нужен как виджет

#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QShowEvent;
class QStackedWidget;
class QTextEdit;
class WorkflowDialogManager;

class WorkerView : public QWidget {
    Q_OBJECT

public:
    explicit WorkerView(WorkerViewModel& viewModel,
                        QWidget*         parent = nullptr);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void showEvent(QShowEvent* event) override;

private slots:
    void onSlotsUpdated(QVector<SlotCellData> items);
    void onOperationStateChanged(const QString& state,
                                 const QString& message);
    void onWorkflowUpdated(const QString& workflow,
                           const QString& order,
                           const QString& checklist,
                           const QString& progress,
                           bool showStartPage);
    void onWorkflowControlsUpdated(const QString& stateKey);
    void onStm32StatusUpdated(const QString& status);
    void onActiveModuleUpdated(const QString& moduleSummary);
    void onActiveModuleAvailabilityChanged(bool online);
    void onErrorOccurred(const QString& message);
    void onImportOrderClicked();
    void onBarcodeSubmitted();
    void onCancelClicked();
    void onStartFeederLoadingClicked();
    void onCompleteFeederLoadingClicked();
    void onArrivedFeederClicked();
    void onStartFeederPrepClicked();
    void onFeederPrepDoneClicked();
    void onArrivedLineClicked();
    void onStartIssuingClicked();
    void onIssueMaterialClicked();
    void onCompleteIssuingClicked();
    void onInspectLeftoversClicked();
    void onStartReturnClicked();
    void onReturnLeftoverClicked();

private:
    WorkerViewModel& viewModel_;
    WorkflowDialogManager* dialogManager_;

    QWidget*        startPage_ = nullptr;
    QWidget*        missingModulePage_ = nullptr;
    QWidget*        workPage_ = nullptr;
    SlotGridWidget* slotGrid_     = nullptr;
    QLabel*         stateLabel_   = nullptr;
    QLabel*         messageLabel_ = nullptr;
    QLabel*         stm32StatusLabel_ = nullptr;
    QLabel*         activeModuleLabel_ = nullptr;
    QLabel*         workflowLabel_ = nullptr;
    QLabel*         workflowPillLabel_ = nullptr;
    QLabel*         actionHintLabel_ = nullptr;
    QLabel*         progressLabel_ = nullptr;
    QLabel*         orderLabel_   = nullptr;
    QTextEdit*      checklistText_ = nullptr;
    QLabel*         errorLabel_   = nullptr;
    QLineEdit*      barcodeEdit_  = nullptr;
    QPushButton*    importButton_ = nullptr;
    QPushButton*    startImportButton_ = nullptr;
    QPushButton*    scanButton_   = nullptr;
    QPushButton*    cancelButton_ = nullptr;
    QPushButton*    startFeederLoadingButton_ = nullptr;
    QPushButton*    completeFeederLoadingButton_ = nullptr;
    QPushButton*    arrivedFeederButton_ = nullptr;
    QPushButton*    startFeederPrepButton_ = nullptr;
    QPushButton*    feederPrepDoneButton_ = nullptr;
    QPushButton*    arrivedLineButton_ = nullptr;
    QPushButton*    startIssuingButton_ = nullptr;
    QPushButton*    issueButton_ = nullptr;
    QPushButton*    completeIssuingButton_ = nullptr;
    QPushButton*    inspectLeftoversButton_ = nullptr;
    QPushButton*    startReturnButton_ = nullptr;
    QPushButton*    returnLeftoverButton_ = nullptr;
    // ── Экраны по типу модуля ────────────────────────────────────────────────
    QLabel*         moduleKindBannerLabel_ = nullptr;
    QStackedWidget* moduleWorkStack_       = nullptr;
    QWidget*        feederWorkPage_        = nullptr;
    QWidget*        reelWorkPage_          = nullptr;

    bool            moduleOnline_ = true;
    bool            showStartPage_ = false;
    QString         currentModuleKind_{QStringLiteral("UNKNOWN")};

    void setupUi();
    void connectSignals();
    void focusBarcodeInput();
    void updateVisiblePage();
    void updateModuleKindScreen(const QString& stateKey);
    void updateScanActionText(const QString& stateKey);
    void updateActionHint(const QString& stateKey);
    void setWorkflowActionsEnabled(bool startFeederLoading,
                                   bool completeFeederLoading,
                                   bool arrivedFeeder,
                                   bool startFeederPrep,
                                   bool feederPrepDone,
                                   bool arrivedLine,
                                   bool startIssuing,
                                   bool issue,
                                   bool completeIssuing,
                                   bool inspectLeftovers,
                                   bool startReturn,
                                   bool returnLeftover);
};
