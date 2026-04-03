// ===== src/presentation/qt/views/WorkerView.cpp =====
// Исправлено:
//   - include paths относительные
//   - все строки через QString::fromUtf8
//   - onOperationStateChanged: убрано хрупкое сравнение строк
#include "WorkerView.hpp"

#include <QFont>
#include <QFrame>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

WorkerView::WorkerView(WorkerViewModel& viewModel, QWidget* parent)
    : QWidget(parent)
    , viewModel_(viewModel)
{
    setupUi();
    connectSignals();

    slotGrid_->updateSlots(viewModel_.slotItems());
    stateLabel_->setText(viewModel_.stateLabel());
}

void WorkerView::setupUi() {
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    // ── Статус-панель ──────────────────────────────────────────────────────────
    auto* statusFrame = new QFrame(this);
    statusFrame->setFrameShape(QFrame::StyledPanel);
    statusFrame->setStyleSheet("QFrame { background: #1E1E2E; border-radius: 8px; }");

    auto* statusLayout = new QVBoxLayout(statusFrame);
    statusLayout->setContentsMargins(12, 8, 12, 8);

    stateLabel_ = new QLabel(QString::fromUtf8("Инициализация..."), statusFrame);
    stateLabel_->setFont(QFont("Segoe UI", 14, QFont::Bold));
    stateLabel_->setStyleSheet("color: #CDD6F4;");

    messageLabel_ = new QLabel(QString::fromUtf8("Подождите..."), statusFrame);
    messageLabel_->setFont(QFont("Segoe UI", 10));
    messageLabel_->setStyleSheet("color: #A6ADC8;");

    errorLabel_ = new QLabel("", statusFrame);
    errorLabel_->setFont(QFont("Segoe UI", 10));
    errorLabel_->setStyleSheet("color: #F38BA8;");
    errorLabel_->setVisible(false);

    statusLayout->addWidget(stateLabel_);
    statusLayout->addWidget(messageLabel_);
    statusLayout->addWidget(errorLabel_);
    rootLayout->addWidget(statusFrame);

    // ── Сетка слотов ───────────────────────────────────────────────────────────
    slotGrid_ = new SlotGridWidget(this);
    rootLayout->addWidget(slotGrid_, /*stretch=*/1);

    // ── Панель ввода штрихкода ─────────────────────────────────────────────────
    auto* inputFrame = new QFrame(this);
    inputFrame->setFrameShape(QFrame::StyledPanel);
    inputFrame->setStyleSheet("QFrame { background: #1E1E2E; border-radius: 8px; }");

    auto* inputLayout = new QHBoxLayout(inputFrame);
    inputLayout->setContentsMargins(12, 8, 12, 8);
    inputLayout->setSpacing(8);

    auto* barcodeLabel = new QLabel(QString::fromUtf8("Штрихкод:"), inputFrame);
    barcodeLabel->setStyleSheet("color: #CDD6F4;");
    barcodeLabel->setFont(QFont("Segoe UI", 10));

    barcodeEdit_ = new QLineEdit(inputFrame);
    barcodeEdit_->setPlaceholderText(
        QString::fromUtf8("Отсканируйте или введите вручную..."));
    barcodeEdit_->setFont(QFont("Segoe UI", 11));
    barcodeEdit_->setMinimumWidth(280);
    barcodeEdit_->setStyleSheet(
        "QLineEdit { background: #313244; color: #CDD6F4; "
        "border: 1px solid #585B70; border-radius: 4px; padding: 4px 8px; }"
        "QLineEdit:focus { border-color: #89B4FA; }"
    );

    scanButton_ = new QPushButton(QString::fromUtf8("Сканировать"), inputFrame);
    scanButton_->setFont(QFont("Segoe UI", 10));
    scanButton_->setStyleSheet(
        "QPushButton { background: #89B4FA; color: #1E1E2E; "
        "border-radius: 4px; padding: 6px 16px; font-weight: bold; }"
        "QPushButton:hover { background: #B4BEFE; }"
        "QPushButton:pressed { background: #74C7EC; }"
    );

    cancelButton_ = new QPushButton(QString::fromUtf8("Отмена"), inputFrame);
    cancelButton_->setFont(QFont("Segoe UI", 10));
    cancelButton_->setEnabled(false);
    cancelButton_->setStyleSheet(
        "QPushButton { background: #F38BA8; color: #1E1E2E; "
        "border-radius: 4px; padding: 6px 16px; font-weight: bold; }"
        "QPushButton:hover { background: #EBA0AC; }"
        "QPushButton:disabled { background: #45475A; color: #585B70; }"
    );

    inputLayout->addWidget(barcodeLabel);
    inputLayout->addWidget(barcodeEdit_, /*stretch=*/1);
    inputLayout->addWidget(scanButton_);
    inputLayout->addWidget(cancelButton_);
    rootLayout->addWidget(inputFrame);

    setLayout(rootLayout);
    setStyleSheet("WorkerView { background: #181825; }");

    // Автофокус на поле ввода при старте
    barcodeEdit_->setFocus();
}

void WorkerView::connectSignals() {
    connect(&viewModel_, &WorkerViewModel::slotsUpdated,
            this, &WorkerView::onSlotsUpdated);
    connect(&viewModel_, &WorkerViewModel::operationStateChanged,
            this, &WorkerView::onOperationStateChanged);
    connect(&viewModel_, &WorkerViewModel::errorOccurred,
            this, &WorkerView::onErrorOccurred);

    connect(scanButton_,  &QPushButton::clicked,
            this, &WorkerView::onBarcodeSubmitted);
    connect(barcodeEdit_, &QLineEdit::returnPressed,
            this, &WorkerView::onBarcodeSubmitted);
    connect(cancelButton_, &QPushButton::clicked,
            this, &WorkerView::onCancelClicked);

    // Управление кнопками по состоянию автомата
    connect(&viewModel_.stateMachineRef(),  
            &smartcart::application::AppStateMachine::stateChanged,
            this, [this](smartcart::application::AppState state) {
                const bool operating =
                    (state == smartcart::application::AppState::Operating);
                cancelButton_->setEnabled(operating);
                scanButton_->setEnabled(!operating);
                barcodeEdit_->setEnabled(!operating);
                if (!operating) barcodeEdit_->setFocus();  // фокус после завершения операции
                // Скрываем ошибку при переходе в рабочее состояние
                if (state == smartcart::application::AppState::Ready ||
                    state == smartcart::application::AppState::Operating)
                {
                    errorLabel_->setVisible(false);
                    errorLabel_->clear();
                }
            });
}

void WorkerView::onSlotsUpdated(QVector<SlotCellData> items) {
    slotGrid_->updateSlots(items);
}

void WorkerView::onOperationStateChanged(const QString& state,
                                         const QString& message) {
    stateLabel_->setText(state);
    messageLabel_->setText(message);
}

void WorkerView::onErrorOccurred(const QString& message) {
    errorLabel_->setText("⚠ " + message);
    errorLabel_->setVisible(true);
}

void WorkerView::onBarcodeSubmitted() {
    const QString barcode = barcodeEdit_->text().trimmed();
    if (barcode.isEmpty()) return;
    barcodeEdit_->clear();
    viewModel_.onBarcodeScanned(barcode);
    barcodeEdit_->setFocus();  // возвращаем фокус после скана
}

void WorkerView::onCancelClicked() {
    viewModel_.cancelCurrentOperation();
}
