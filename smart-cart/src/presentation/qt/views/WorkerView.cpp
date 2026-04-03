// ===== src/presentation/qt/views/WorkerView.cpp =====
// Исправлено:
//   - include paths относительные
//   - все строки через QString::fromUtf8
//   - onOperationStateChanged: убрано хрупкое сравнение строк
#include "WorkerView.hpp"

#include <QApplication>
#include <QEvent>
#include <QFont>
#include <QFileDialog>
#include <QFrame>
#include <QGridLayout>
#include <QKeyEvent>
#include <QSizePolicy>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>

WorkerView::WorkerView(WorkerViewModel& viewModel, QWidget* parent)
    : QWidget(parent)
    , viewModel_(viewModel)
{
    setupUi();
    connectSignals();
    qApp->installEventFilter(this);

    slotGrid_->updateSlots(viewModel_.slotItems());
    stateLabel_->setText(viewModel_.stateLabel());
    viewModel_.reload();
}

bool WorkerView::eventFilter(QObject* watched, QEvent* event) {
    if (!barcodeEdit_ ||
        event->type() != QEvent::KeyPress ||
        QApplication::activeWindow() != window())
    {
        return QWidget::eventFilter(watched, event);
    }
    if (workPage_ && !workPage_->isVisible()) {
        return QWidget::eventFilter(watched, event);
    }

    auto* keyEvent = static_cast<QKeyEvent*>(event);
    const auto modifiers = keyEvent->modifiers();
    if (modifiers.testFlag(Qt::ControlModifier) ||
        modifiers.testFlag(Qt::AltModifier) ||
        modifiers.testFlag(Qt::MetaModifier))
    {
        return QWidget::eventFilter(watched, event);
    }

    auto* focused = QApplication::focusWidget();
    if (focused == barcodeEdit_) {
        return QWidget::eventFilter(watched, event);
    }
    if (focused && !isAncestorOf(focused)) {
        return QWidget::eventFilter(watched, event);
    }

    if (keyEvent->key() == Qt::Key_Return ||
        keyEvent->key() == Qt::Key_Enter)
    {
        if (!barcodeEdit_->text().trimmed().isEmpty()) {
            onBarcodeSubmitted();
            return true;
        }
        focusBarcodeInput();
        return QWidget::eventFilter(watched, event);
    }

    const QString text = keyEvent->text();
    if (text.isEmpty() || !text.at(0).isPrint()) {
        return QWidget::eventFilter(watched, event);
    }

    focusBarcodeInput();
    barcodeEdit_->insert(text);
    return true;
}

void WorkerView::setupUi() {
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    const QString panelStyle =
        "QFrame { background: #F6FAF7; border: 1px solid #D5E2D8; "
        "border-radius: 8px; }";
    const QString darkPanelStyle =
        "QFrame { background: #122A1E; border: 1px solid #31533D; "
        "border-radius: 8px; }";
    const QString primaryButtonStyle =
        "QPushButton { background: #2F8F57; color: #FFFFFF; "
        "border: 1px solid #2F8F57; border-radius: 8px; "
        "padding: 10px 14px; font-weight: bold; }"
        "QPushButton:hover { background: #3CA86A; }"
        "QPushButton:pressed { background: #267346; }"
        "QPushButton:disabled { background: #DCE5DF; color: #819086; "
        "border-color: #DCE5DF; }";
    const QString secondaryButtonStyle =
        "QPushButton { background: #E6F0E9; color: #173025; "
        "border: 1px solid #B8D0BF; border-radius: 8px; "
        "padding: 9px 12px; font-weight: bold; }"
        "QPushButton:hover { background: #D7E8DC; }"
        "QPushButton:pressed { background: #C4DACB; }"
        "QPushButton:disabled { background: #E7ECE8; color: #8B968F; "
        "border-color: #D6DED8; }";
    const QString dangerButtonStyle =
        "QPushButton { background: #B85C5C; color: #FFFFFF; "
        "border: 1px solid #B85C5C; border-radius: 8px; "
        "padding: 9px 12px; font-weight: bold; }"
        "QPushButton:hover { background: #C86F6F; }"
        "QPushButton:pressed { background: #954747; }"
        "QPushButton:disabled { background: #E7ECE8; color: #8B968F; "
        "border-color: #D6DED8; }";

    startPage_ = new QWidget(this);
    startPage_->setStyleSheet("QWidget { background: #102217; }");
    auto* startLayout = new QVBoxLayout(startPage_);
    startLayout->setContentsMargins(40, 32, 40, 32);
    startLayout->setSpacing(18);
    startLayout->addStretch();

    auto* startCard = new QFrame(startPage_);
    startCard->setFrameShape(QFrame::StyledPanel);
    startCard->setMinimumSize(620, 280);
    startCard->setMaximumWidth(820);
    startCard->setStyleSheet(
        "QFrame { background: #F6FAF7; border: 1px solid #AFC8B8; "
        "border-radius: 8px; }");

    auto* startCardLayout = new QVBoxLayout(startCard);
    startCardLayout->setContentsMargins(28, 26, 28, 26);
    startCardLayout->setSpacing(16);

    auto* startTitle = new QLabel(QString::fromUtf8("Тележка свободна"), startCard);
    startTitle->setFont(QFont("Segoe UI", 26, QFont::Bold));
    startTitle->setStyleSheet("color: #102217;");
    startTitle->setAlignment(Qt::AlignCenter);

    auto* startText = new QLabel(
        QString::fromUtf8("1. Остатков нет.\n2. Тележка свободна. Выберите заказ."),
        startCard);
    startText->setFont(QFont("Segoe UI", 15));
    startText->setStyleSheet("color: #31533D;");
    startText->setAlignment(Qt::AlignCenter);
    startText->setWordWrap(true);

    startImportButton_ =
        new QPushButton(QString::fromUtf8("Выбрать заказ"), startCard);
    startImportButton_->setFont(QFont("Segoe UI", 15, QFont::Bold));
    startImportButton_->setMinimumSize(220, 56);
    startImportButton_->setStyleSheet(primaryButtonStyle);

    startCardLayout->addWidget(startTitle);
    startCardLayout->addWidget(startText);
    startCardLayout->addWidget(startImportButton_, 0, Qt::AlignCenter);
    startLayout->addWidget(startCard, 0, Qt::AlignCenter);
    startLayout->addStretch();

    missingModulePage_ = new QWidget(this);
    missingModulePage_->setStyleSheet("QWidget { background: #102217; }");
    auto* missingLayout = new QVBoxLayout(missingModulePage_);
    missingLayout->setContentsMargins(40, 32, 40, 32);
    missingLayout->setSpacing(18);
    missingLayout->addStretch();

    auto* missingCard = new QFrame(missingModulePage_);
    missingCard->setFrameShape(QFrame::StyledPanel);
    missingCard->setMinimumSize(620, 280);
    missingCard->setMaximumWidth(820);
    missingCard->setStyleSheet(
        "QFrame { background: #F6FAF7; border: 1px solid #AFC8B8; "
        "border-radius: 8px; }");

    auto* missingCardLayout = new QVBoxLayout(missingCard);
    missingCardLayout->setContentsMargins(28, 26, 28, 26);
    missingCardLayout->setSpacing(16);

    auto* missingTitle = new QLabel(QString::fromUtf8("Вставьте модуль"), missingCard);
    missingTitle->setFont(QFont("Segoe UI", 26, QFont::Bold));
    missingTitle->setStyleSheet("color: #102217;");
    missingTitle->setAlignment(Qt::AlignCenter);

    auto* missingText = new QLabel(
        QString::fromUtf8(
            "1. Активный модуль сейчас недоступен.\n"
            "2. Вставьте модуль в тележку и поднесите RFID-метку к считывателю.\n"
            "3. После перехода модуля в онлайн рабочий экран откроется автоматически."),
        missingCard);
    missingText->setFont(QFont("Segoe UI", 15));
    missingText->setStyleSheet("color: #31533D;");
    missingText->setAlignment(Qt::AlignCenter);
    missingText->setWordWrap(true);

    missingCardLayout->addWidget(missingTitle);
    missingCardLayout->addWidget(missingText);
    missingLayout->addWidget(missingCard, 0, Qt::AlignCenter);
    missingLayout->addStretch();

    workPage_ = new QWidget(this);
    workPage_->setStyleSheet("QWidget { background: #102217; }");
    auto* workLayout = new QVBoxLayout(workPage_);
    workLayout->setContentsMargins(10, 10, 10, 10);
    workLayout->setSpacing(10);

    rootLayout->addWidget(startPage_);
    rootLayout->addWidget(missingModulePage_);
    rootLayout->addWidget(workPage_);

    // ── Верхняя панель: крупная подсказка для оператора ─────────────────────
    auto* statusFrame = new QFrame(workPage_);
    statusFrame->setFrameShape(QFrame::StyledPanel);
    statusFrame->setStyleSheet(darkPanelStyle);

    auto* statusLayout = new QHBoxLayout(statusFrame);
    statusLayout->setContentsMargins(14, 10, 14, 10);
    statusLayout->setSpacing(12);

    auto* statusTextLayout = new QVBoxLayout();
    statusTextLayout->setSpacing(4);

    workflowPillLabel_ = new QLabel(QString::fromUtf8("Тележка свободна"),
                                    statusFrame);
    workflowPillLabel_->setFont(QFont("Segoe UI", 10, QFont::Bold));
    workflowPillLabel_->setStyleSheet(
        "QLabel { background: #E6F4EA; color: #1F6F43; "
        "border: 1px solid #A5D3B4; border-radius: 8px; padding: 4px 10px; }"
    );

    stateLabel_ = new QLabel(QString::fromUtf8("Инициализация..."), statusFrame);
    stateLabel_->setFont(QFont("Segoe UI", 19, QFont::Bold));
    stateLabel_->setStyleSheet("color: #FFFFFF;");

    messageLabel_ = new QLabel(QString::fromUtf8("Подождите..."), statusFrame);
    messageLabel_->setFont(QFont("Segoe UI", 12));
    messageLabel_->setStyleSheet("color: #C8D9CE;");
    messageLabel_->setWordWrap(true);

    actionHintLabel_ = new QLabel(
        QString::fromUtf8("Загрузите заказ, чтобы начать подбор материалов"),
        statusFrame);
    actionHintLabel_->setFont(QFont("Segoe UI", 13, QFont::Bold));
    actionHintLabel_->setWordWrap(true);
    actionHintLabel_->setStyleSheet("color: #8DE4AD;");

    activeModuleLabel_ = new QLabel(
        QString::fromUtf8("Активный модуль: определяем..."),
        statusFrame);
    activeModuleLabel_->setFont(QFont("Segoe UI", 11, QFont::Bold));
    activeModuleLabel_->setWordWrap(true);
    activeModuleLabel_->setTextFormat(Qt::RichText);
    activeModuleLabel_->setStyleSheet(
        "QLabel { color: #DDF3E4; background: #1A3B2A; "
        "border: 1px solid #3E6B4C; border-radius: 8px; padding: 6px 10px; }"
    );

    stm32StatusLabel_ = new QLabel(QString::fromUtf8("STM32: ожидание связи"),
                                   statusFrame);
    stm32StatusLabel_->setFont(QFont("Segoe UI", 9));
    stm32StatusLabel_->setWordWrap(true);
    stm32StatusLabel_->setMinimumWidth(280);
    stm32StatusLabel_->setStyleSheet(
        "QLabel { background: #183928; color: #DDEBE1; "
        "border: 1px solid #3E6B4C; border-radius: 8px; padding: 8px 10px; }"
    );

    errorLabel_ = new QLabel("", statusFrame);
    errorLabel_->setFont(QFont("Segoe UI", 11, QFont::Bold));
    errorLabel_->setWordWrap(true);
    errorLabel_->setStyleSheet(
        "QLabel { color: #FFE8E8; background: #803B3B; "
        "border: 1px solid #B85C5C; border-radius: 8px; padding: 6px 10px; }"
    );
    errorLabel_->setVisible(false);

    statusTextLayout->addWidget(workflowPillLabel_, 0, Qt::AlignLeft);
    statusTextLayout->addWidget(stateLabel_);
    statusTextLayout->addWidget(messageLabel_);
    statusTextLayout->addWidget(actionHintLabel_);
    statusTextLayout->addWidget(activeModuleLabel_);
    statusTextLayout->addWidget(errorLabel_);

    statusLayout->addLayout(statusTextLayout, 1);
    statusLayout->addWidget(stm32StatusLabel_);
    workLayout->addWidget(statusFrame);

    auto* bodyLayout = new QHBoxLayout();
    bodyLayout->setSpacing(10);

    // ── Сетка слотов: главный рабочий объект на сенсорном экране ────────────
    auto* slotFrame = new QFrame(workPage_);
    slotFrame->setFrameShape(QFrame::StyledPanel);
    slotFrame->setStyleSheet(panelStyle);
    auto* slotLayout = new QVBoxLayout(slotFrame);
    slotLayout->setContentsMargins(12, 10, 12, 12);
    slotLayout->setSpacing(8);

    auto* slotTitle = new QLabel(QString::fromUtf8("Слоты тележки"), slotFrame);
    slotTitle->setFont(QFont("Segoe UI", 15, QFont::Bold));
    slotTitle->setStyleSheet("color: #102217;");

    slotGrid_ = new SlotGridWidget(slotFrame);
    slotLayout->addWidget(slotTitle);
    slotLayout->addWidget(slotGrid_, 1);

    auto* rightColumn = new QWidget(workPage_);
    auto* rightLayout = new QVBoxLayout(rightColumn);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(10);

    // ── Панель ввода штрихкода: крупная зона действия пальцем ───────────────
    auto* inputFrame = new QFrame(workPage_);
    inputFrame->setFrameShape(QFrame::StyledPanel);
    inputFrame->setStyleSheet(panelStyle);

    auto* inputLayout = new QGridLayout(inputFrame);
    inputLayout->setContentsMargins(12, 10, 12, 12);
    inputLayout->setHorizontalSpacing(8);
    inputLayout->setVerticalSpacing(8);

    auto* barcodeLabel = new QLabel(QString::fromUtf8("Сканер"), inputFrame);
    barcodeLabel->setStyleSheet("color: #102217;");
    barcodeLabel->setFont(QFont("Segoe UI", 15, QFont::Bold));

    barcodeEdit_ = new QLineEdit(inputFrame);
    barcodeEdit_->setPlaceholderText(
        QString::fromUtf8("Отсканируйте или введите вручную..."));
    barcodeEdit_->setFont(QFont("Segoe UI", 13));
    barcodeEdit_->setMinimumHeight(42);
    barcodeEdit_->setMinimumWidth(260);
    barcodeEdit_->setStyleSheet(
        "QLineEdit { background: #FFFFFF; color: #102217; "
        "border: 2px solid #B8D0BF; border-radius: 8px; padding: 8px 12px; }"
        "QLineEdit:focus { border-color: #2F8F57; }"
    );

    importButton_ = new QPushButton(QString::fromUtf8("Загрузить заказ"),
                                    inputFrame);
    importButton_->setFont(QFont("Segoe UI", 11, QFont::Bold));
    importButton_->setMinimumHeight(40);
    importButton_->setStyleSheet(secondaryButtonStyle);

    scanButton_ = new QPushButton(QString::fromUtf8("Сканировать"), inputFrame);
    scanButton_->setFont(QFont("Segoe UI", 12, QFont::Bold));
    scanButton_->setMinimumHeight(46);
    scanButton_->setStyleSheet(primaryButtonStyle);

    cancelButton_ = new QPushButton(QString::fromUtf8("Отмена"), inputFrame);
    cancelButton_->setFont(QFont("Segoe UI", 11, QFont::Bold));
    cancelButton_->setMinimumHeight(40);
    cancelButton_->setEnabled(false);
    cancelButton_->setStyleSheet(dangerButtonStyle);

    inputLayout->addWidget(barcodeLabel, 0, 0, 1, 2);
    inputLayout->addWidget(barcodeEdit_, 1, 0, 1, 2);
    inputLayout->addWidget(scanButton_, 2, 0, 1, 2);
    inputLayout->addWidget(importButton_, 3, 0);
    inputLayout->addWidget(cancelButton_, 3, 1);
    rightLayout->addWidget(inputFrame);

    // ── Заказ и чек-лист ─────────────────────────────────────────────────────
    auto* orderFrame = new QFrame(workPage_);
    orderFrame->setFrameShape(QFrame::StyledPanel);
    orderFrame->setStyleSheet(panelStyle);

    auto* orderLayout = new QVBoxLayout(orderFrame);
    orderLayout->setContentsMargins(12, 10, 12, 12);
    orderLayout->setSpacing(6);

    workflowLabel_ = new QLabel(QString::fromUtf8("Заказ"), orderFrame);
    workflowLabel_->setFont(QFont("Segoe UI", 15, QFont::Bold));
    workflowLabel_->setStyleSheet("color: #102217;");

    orderLabel_ = new QLabel(QString::fromUtf8("Заказ не загружен"), orderFrame);
    orderLabel_->setFont(QFont("Segoe UI", 11));
    orderLabel_->setStyleSheet("color: #102217;");
    orderLabel_->setWordWrap(true);
    orderLabel_->setTextFormat(Qt::RichText);

    progressLabel_ = new QLabel(QString::fromUtf8("Материалы: 0/0"), orderFrame);
    progressLabel_->setFont(QFont("Segoe UI", 11, QFont::Bold));
    progressLabel_->setWordWrap(true);
    progressLabel_->setStyleSheet("color: #31533D;");

    checklistText_ = new QTextEdit(orderFrame);
    checklistText_->setReadOnly(true);
    checklistText_->setMinimumHeight(110);
    checklistText_->setFont(QFont("Segoe UI", 11));
    checklistText_->setStyleSheet(
        "QTextEdit { background: #FFFFFF; color: #102217; "
        "border: 1px solid #C7D8CC; border-radius: 8px; padding: 8px; }"
    );
    checklistText_->setPlainText(QString::fromUtf8("Загрузите JSON заказа"));

    orderLayout->addWidget(workflowLabel_);
    orderLayout->addWidget(orderLabel_);
    orderLayout->addWidget(progressLabel_);
    orderLayout->addWidget(checklistText_, 1);
    rightLayout->addWidget(orderFrame, 1);

    // ── Этапы маршрута тележки ───────────────────────────────────────────────
    auto* workflowFrame = new QFrame(workPage_);
    workflowFrame->setFrameShape(QFrame::StyledPanel);
    workflowFrame->setStyleSheet(panelStyle);

    auto* workflowRootLayout = new QVBoxLayout(workflowFrame);
    workflowRootLayout->setContentsMargins(12, 10, 12, 12);
    workflowRootLayout->setSpacing(6);

    auto* routeTitle = new QLabel(QString::fromUtf8("Маршрут тележки"),
                                  workflowFrame);
    routeTitle->setFont(QFont("Segoe UI", 15, QFont::Bold));
    routeTitle->setStyleSheet("color: #102217;");

    auto* workflowLayout = new QGridLayout();
    workflowLayout->setHorizontalSpacing(8);
    workflowLayout->setVerticalSpacing(8);

    auto makeWorkflowButton = [workflowFrame, secondaryButtonStyle](const QString& text) {
        auto* button = new QPushButton(text, workflowFrame);
        button->setFont(QFont("Segoe UI", 10, QFont::Bold));
        button->setMinimumHeight(38);
        button->setStyleSheet(secondaryButtonStyle);
        return button;
    };

    arrivedFeederButton_ =
        makeWorkflowButton(QString::fromUtf8("Прибыла к питателям"));
    startFeederPrepButton_ =
        makeWorkflowButton(QString::fromUtf8("Начать подготовку"));
    feederPrepDoneButton_ =
        makeWorkflowButton(QString::fromUtf8("Питатели готовы"));
    arrivedLineButton_ =
        makeWorkflowButton(QString::fromUtf8("Прибыла на линию"));
    startIssuingButton_ =
        makeWorkflowButton(QString::fromUtf8("Начать выдачу"));
    issueButton_ =
        makeWorkflowButton(QString::fromUtf8("Выдать штрихкод"));
    completeIssuingButton_ =
        makeWorkflowButton(QString::fromUtf8("Завершить выдачу"));
    inspectLeftoversButton_ =
        makeWorkflowButton(QString::fromUtf8("Проверить остатки"));
    startReturnButton_ =
        makeWorkflowButton(QString::fromUtf8("Начать возврат"));
    returnLeftoverButton_ =
        makeWorkflowButton(QString::fromUtf8("Вернуть остаток"));

    workflowLayout->addWidget(arrivedFeederButton_, 0, 0);
    workflowLayout->addWidget(startFeederPrepButton_, 0, 1);
    workflowLayout->addWidget(feederPrepDoneButton_, 0, 2);
    workflowLayout->addWidget(arrivedLineButton_, 1, 0);
    workflowLayout->addWidget(startIssuingButton_, 1, 1);
    workflowLayout->addWidget(issueButton_, 1, 2);
    workflowLayout->addWidget(completeIssuingButton_, 2, 0);
    workflowLayout->addWidget(inspectLeftoversButton_, 2, 1);
    workflowLayout->addWidget(startReturnButton_, 2, 2);
    workflowLayout->addWidget(returnLeftoverButton_, 3, 0, 1, 3);
    workflowRootLayout->addWidget(routeTitle);
    workflowRootLayout->addLayout(workflowLayout);
    rightLayout->addWidget(workflowFrame);
    rightLayout->addStretch();

    auto* rightScroll = new QScrollArea(workPage_);
    rightScroll->setFrameShape(QFrame::NoFrame);
    rightScroll->setWidgetResizable(true);
    rightScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    rightScroll->setWidget(rightColumn);

    bodyLayout->addWidget(slotFrame, 7);
    bodyLayout->addWidget(rightScroll, 5);
    workLayout->addLayout(bodyLayout, 1);

    setLayout(rootLayout);
    setStyleSheet("WorkerView { background: #102217; }");

    // Автофокус на поле ввода при старте
    barcodeEdit_->setFocus();
}

void WorkerView::connectSignals() {
    connect(&viewModel_, &WorkerViewModel::slotsUpdated,
            this, &WorkerView::onSlotsUpdated);
    connect(&viewModel_, &WorkerViewModel::operationStateChanged,
            this, &WorkerView::onOperationStateChanged);
    connect(&viewModel_, &WorkerViewModel::workflowUpdated,
            this, &WorkerView::onWorkflowUpdated);
    connect(&viewModel_, &WorkerViewModel::workflowControlsUpdated,
            this, &WorkerView::onWorkflowControlsUpdated);
    connect(&viewModel_, &WorkerViewModel::stm32StatusUpdated,
            this, &WorkerView::onStm32StatusUpdated);
    connect(&viewModel_, &WorkerViewModel::activeModuleUpdated,
            this, &WorkerView::onActiveModuleUpdated);
    connect(&viewModel_, &WorkerViewModel::activeModuleAvailabilityChanged,
            this, &WorkerView::onActiveModuleAvailabilityChanged);
    connect(&viewModel_, &WorkerViewModel::errorOccurred,
            this, &WorkerView::onErrorOccurred);

    connect(importButton_, &QPushButton::clicked,
            this, &WorkerView::onImportOrderClicked);
    connect(startImportButton_, &QPushButton::clicked,
            this, &WorkerView::onImportOrderClicked);
    connect(scanButton_,  &QPushButton::clicked,
            this, &WorkerView::onBarcodeSubmitted);
    connect(barcodeEdit_, &QLineEdit::returnPressed,
            this, &WorkerView::onBarcodeSubmitted);
    connect(cancelButton_, &QPushButton::clicked,
            this, &WorkerView::onCancelClicked);
    connect(arrivedFeederButton_, &QPushButton::clicked,
            this, &WorkerView::onArrivedFeederClicked);
    connect(startFeederPrepButton_, &QPushButton::clicked,
            this, &WorkerView::onStartFeederPrepClicked);
    connect(feederPrepDoneButton_, &QPushButton::clicked,
            this, &WorkerView::onFeederPrepDoneClicked);
    connect(arrivedLineButton_, &QPushButton::clicked,
            this, &WorkerView::onArrivedLineClicked);
    connect(startIssuingButton_, &QPushButton::clicked,
            this, &WorkerView::onStartIssuingClicked);
    connect(issueButton_, &QPushButton::clicked,
            this, &WorkerView::onIssueMaterialClicked);
    connect(completeIssuingButton_, &QPushButton::clicked,
            this, &WorkerView::onCompleteIssuingClicked);
    connect(inspectLeftoversButton_, &QPushButton::clicked,
            this, &WorkerView::onInspectLeftoversClicked);
    connect(startReturnButton_, &QPushButton::clicked,
            this, &WorkerView::onStartReturnClicked);
    connect(returnLeftoverButton_, &QPushButton::clicked,
            this, &WorkerView::onReturnLeftoverClicked);

    // Управление кнопками по состоянию автомата
    connect(&viewModel_.stateMachineRef(),  
            &smartcart::application::AppStateMachine::stateChanged,
            this, [this](smartcart::application::AppState state) {
                const bool operating =
                    (state == smartcart::application::AppState::Operating);
                cancelButton_->setEnabled(operating);
                scanButton_->setEnabled(!operating && moduleOnline_);
                barcodeEdit_->setEnabled(!operating && moduleOnline_);
                if (!operating) barcodeEdit_->setFocus();  // фокус после завершения операции
                // Скрываем ошибку при переходе в рабочее состояние
                if (state == smartcart::application::AppState::Ready ||
                    state == smartcart::application::AppState::Operating)
                {
                    errorLabel_->setVisible(false);
                    errorLabel_->clear();
                }
            });

    auto* reloadTimer = new QTimer(this);
    connect(reloadTimer, &QTimer::timeout,
            &viewModel_, &WorkerViewModel::reload);
    reloadTimer->start(1000);
    onWorkflowControlsUpdated(QStringLiteral("FREE"));
}

void WorkerView::onSlotsUpdated(QVector<SlotCellData> items) {
    slotGrid_->updateSlots(items);
}

void WorkerView::onOperationStateChanged(const QString& state,
                                         const QString& message) {
    stateLabel_->setText(state);
    messageLabel_->setText(message);
}

void WorkerView::onWorkflowUpdated(const QString& workflow,
                                   const QString& order,
                                   const QString& checklist,
                                   const QString& progress,
                                   bool showStartPage) {
    workflowPillLabel_->setText(workflow);
    workflowLabel_->setText(workflow);
    orderLabel_->setText(order);
    progressLabel_->setText(progress);
    checklistText_->setPlainText(checklist);

    showStartPage_ = showStartPage;
    updateVisiblePage();
}

void WorkerView::onStm32StatusUpdated(const QString& status) {
    stm32StatusLabel_->setText(QString::fromUtf8("STM32: ") + status);
}

void WorkerView::onActiveModuleUpdated(const QString& moduleSummary) {
    if (activeModuleLabel_) {
        activeModuleLabel_->setText(moduleSummary);
    }
}

void WorkerView::onActiveModuleAvailabilityChanged(bool online) {
    moduleOnline_ = online;
    updateVisiblePage();
}

void WorkerView::onWorkflowControlsUpdated(const QString& stateKey) {
    if (stateKey == QStringLiteral("READY_FOR_FEEDER_PREP")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(true, true, false, false, false,
                                  false, false, false, false, false);
        focusBarcodeInput();
        return;
    }

    if (stateKey == QStringLiteral("FEEDER_PREP")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(false, false, true, false, false,
                                  false, false, false, false, false);
        focusBarcodeInput();
        return;
    }

    if (stateKey == QStringLiteral("READY_FOR_LINE")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(false, false, false, true, true,
                                  false, false, false, false, false);
        focusBarcodeInput();
        return;
    }

    if (stateKey == QStringLiteral("ISSUING_TO_LINE")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(false, false, false, false, false,
                                  true, true, false, false, false);
        focusBarcodeInput();
        return;
    }

    if (stateKey == QStringLiteral("ORDER_COMPLETED")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(false, false, false, false, false,
                                  false, false, true, false, false);
        focusBarcodeInput();
        return;
    }

    if (stateKey == QStringLiteral("LEFTOVERS_DETECTED")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(false, false, false, false, false,
                                  false, false, true, true, true);
        focusBarcodeInput();
        return;
    }

    if (stateKey == QStringLiteral("RETURNING_LEFTOVERS")) {
        updateScanActionText(stateKey);
        updateActionHint(stateKey);
        setWorkflowActionsEnabled(false, false, false, false, false,
                                  false, false, true, false, true);
        focusBarcodeInput();
        return;
    }

    updateScanActionText(stateKey);
    updateActionHint(stateKey);
    setWorkflowActionsEnabled(false, false, false, false, false,
                              false, false, false, false, false);
    focusBarcodeInput();
}

void WorkerView::onErrorOccurred(const QString& message) {
    errorLabel_->setText(QString::fromUtf8("Ошибка: ") + message);
    errorLabel_->setVisible(true);
}

void WorkerView::onImportOrderClicked() {
    const QString path = QFileDialog::getOpenFileName(
        this,
        QString::fromUtf8("Выберите JSON заказа"),
        QString(),
        QString::fromUtf8("JSON (*.json);;Все файлы (*)")
    );
    if (path.isEmpty()) {
        return;
    }
    viewModel_.importOrderFromFile(path);
    focusBarcodeInput();
}

void WorkerView::onBarcodeSubmitted() {
    const QString barcode = barcodeEdit_->text().trimmed();
    if (barcode.isEmpty()) return;
    barcodeEdit_->clear();
    viewModel_.submitBarcode(barcode);
    barcodeEdit_->setFocus();  // возвращаем фокус после скана
}

void WorkerView::onCancelClicked() {
    viewModel_.cancelCurrentOperation();
    focusBarcodeInput();
}

void WorkerView::onArrivedFeederClicked() {
    viewModel_.markCartArrivedToFeederPrep();
    focusBarcodeInput();
}

void WorkerView::onStartFeederPrepClicked() {
    viewModel_.startFeederPrep();
    focusBarcodeInput();
}

void WorkerView::onFeederPrepDoneClicked() {
    viewModel_.markFeederPrepCompleted();
    focusBarcodeInput();
}

void WorkerView::onArrivedLineClicked() {
    viewModel_.markCartArrivedToLine();
    focusBarcodeInput();
}

void WorkerView::onStartIssuingClicked() {
    viewModel_.startIssuingToLine();
    focusBarcodeInput();
}

void WorkerView::onIssueMaterialClicked() {
    viewModel_.markItemIssued(barcodeEdit_->text().trimmed());
    barcodeEdit_->clear();
    focusBarcodeInput();
}

void WorkerView::onCompleteIssuingClicked() {
    viewModel_.completeIssuing();
    focusBarcodeInput();
}

void WorkerView::onInspectLeftoversClicked() {
    viewModel_.inspectLeftovers();
    focusBarcodeInput();
}

void WorkerView::onStartReturnClicked() {
    viewModel_.startReturningLeftovers();
    focusBarcodeInput();
}

void WorkerView::onReturnLeftoverClicked() {
    viewModel_.markLeftoverReturned(barcodeEdit_->text().trimmed());
    barcodeEdit_->clear();
    focusBarcodeInput();
}

void WorkerView::focusBarcodeInput() {
    if (!barcodeEdit_ ||
        !barcodeEdit_->isEnabled() ||
        (workPage_ && !workPage_->isVisible()) ||
        QApplication::activeWindow() != window())
    {
        return;
    }
    barcodeEdit_->setFocus(Qt::OtherFocusReason);
}

void WorkerView::updateVisiblePage() {
    if (!startPage_ || !missingModulePage_ || !workPage_) {
        return;
    }

    const bool showMissingModulePage = !moduleOnline_;
    startPage_->setVisible(!showMissingModulePage && showStartPage_);
    missingModulePage_->setVisible(showMissingModulePage);
    workPage_->setVisible(!showMissingModulePage && !showStartPage_);

    const bool workEnabled = !showMissingModulePage;
    if (barcodeEdit_) {
        barcodeEdit_->setEnabled(workEnabled);
    }
    if (scanButton_) {
        scanButton_->setEnabled(workEnabled);
    }
    if (importButton_) {
        importButton_->setEnabled(workEnabled);
    }
    if (startImportButton_) {
        startImportButton_->setEnabled(workEnabled);
    }
    if (cancelButton_) {
        if (!workEnabled) {
            cancelButton_->setEnabled(false);
        }
    }
}

void WorkerView::updateScanActionText(const QString& stateKey) {
    if (stateKey == QStringLiteral("ISSUING_TO_LINE")) {
        scanButton_->setText(QString::fromUtf8("Выдать"));
        barcodeEdit_->setPlaceholderText(
            QString::fromUtf8("Отсканируйте катушку для выдачи на линию..."));
        return;
    }

    if (stateKey == QStringLiteral("LEFTOVERS_DETECTED") ||
        stateKey == QStringLiteral("RETURNING_LEFTOVERS"))
    {
        scanButton_->setText(QString::fromUtf8("Вернуть"));
        barcodeEdit_->setPlaceholderText(
            QString::fromUtf8("Отсканируйте остаток или введите номер слота..."));
        return;
    }

    scanButton_->setText(QString::fromUtf8("Сканировать"));
    barcodeEdit_->setPlaceholderText(
        QString::fromUtf8("Отсканируйте или введите вручную..."));
}

void WorkerView::updateActionHint(const QString& stateKey) {
    if (!actionHintLabel_) {
        return;
    }

    if (stateKey == QStringLiteral("FREE")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Загрузите JSON заказа, чтобы начать подбор"));
        return;
    }

    if (stateKey == QStringLiteral("ORDER_LOADED") ||
        stateKey == QStringLiteral("PICKING_MATERIALS"))
    {
        actionHintLabel_->setText(
            QString::fromUtf8("Сканируйте материал и кладите его в подсказанный слот"));
        return;
    }

    if (stateKey == QStringLiteral("READY_FOR_FEEDER_PREP")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Перевезите тележку к зоне подготовки питателей"));
        return;
    }

    if (stateKey == QStringLiteral("FEEDER_PREP")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Подготовьте питатели и подтвердите завершение этапа"));
        return;
    }

    if (stateKey == QStringLiteral("READY_FOR_LINE")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Перевезите тележку к линии SMT"));
        return;
    }

    if (stateKey == QStringLiteral("ISSUING_TO_LINE")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Сканируйте катушки при выдаче на линию"));
        return;
    }

    if (stateKey == QStringLiteral("ORDER_COMPLETED")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Проверьте остатки после завершения заказа"));
        return;
    }

    if (stateKey == QStringLiteral("LEFTOVERS_DETECTED")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Верните остатки перед загрузкой нового заказа"));
        return;
    }

    if (stateKey == QStringLiteral("RETURNING_LEFTOVERS")) {
        actionHintLabel_->setText(
            QString::fromUtf8("Сканируйте остаток или введите номер слота"));
        return;
    }

    actionHintLabel_->setText(QString::fromUtf8("Следуйте текущему этапу"));
}

void WorkerView::setWorkflowActionsEnabled(bool arrivedFeeder,
                                           bool startFeederPrep,
                                           bool feederPrepDone,
                                           bool arrivedLine,
                                           bool startIssuing,
                                           bool issue,
                                           bool completeIssuing,
                                           bool inspectLeftovers,
                                           bool startReturn,
                                           bool returnLeftover) {
    arrivedFeederButton_->setEnabled(arrivedFeeder);
    startFeederPrepButton_->setEnabled(startFeederPrep);
    feederPrepDoneButton_->setEnabled(feederPrepDone);
    arrivedLineButton_->setEnabled(arrivedLine);
    startIssuingButton_->setEnabled(startIssuing);
    issueButton_->setEnabled(issue);
    completeIssuingButton_->setEnabled(completeIssuing);
    inspectLeftoversButton_->setEnabled(inspectLeftovers);
    startReturnButton_->setEnabled(startReturn);
    returnLeftoverButton_->setEnabled(returnLeftover);
}
