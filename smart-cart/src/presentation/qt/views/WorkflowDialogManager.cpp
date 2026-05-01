#include "WorkflowDialogManager.hpp"
#include "presentation/qt/viewmodels/WorkerViewModel.hpp"

#include <QDialog>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

WorkflowDialogManager::WorkflowDialogManager(QWidget* parent, WorkerViewModel& viewModel)
    : QObject(parent)
    , parent_(parent)
    , viewModel_(viewModel)
{}

void WorkflowDialogManager::setModalPopupsReady(bool ready) {
    modalPopupsReady_ = ready;
}

void WorkflowDialogManager::setModuleOnline(bool online) {
    moduleOnline_ = online;
}

void WorkflowDialogManager::showStagePopupIfNeeded(const QString& stateKey) {
    if (stateKey == QStringLiteral("FREE")) {
        lastStagePopupKey_.clear();
        return;
    }
    if (!moduleOnline_ || stateKey == lastStagePopupKey_) {
        return;
    }

    const QString title = stagePopupTitle(stateKey);
    if (title.isEmpty()) {
        lastStagePopupKey_ = stateKey;
        return;
    }

    lastStagePopupKey_ = stateKey;

    if (!modalPopupsReady_) {
        return;
    }

    QTimer::singleShot(500, this, [this, stateKey, title]() {
        if (!modalPopupsReady_ ||
            !moduleOnline_ ||
            stateKey != lastStagePopupKey_)
        {
            return;
        }
        auto* topLevel = parent_->window();
        if (!topLevel || !topLevel->isVisible() || !parent_->isVisible()) {
            lastStagePopupKey_.clear();
            return;
        }
        showLargeStageDialog(title);
    });
}

QString WorkflowDialogManager::stagePopupTitle(const QString& stateKey) {
    if (stateKey == QStringLiteral("ORDER_LOADED")) {
        return QString::fromUtf8("ЗАКАЗ ЗАГРУЖЕН");
    }
    if (stateKey == QStringLiteral("LOADING_FEEDERS")) {
        return QString::fromUtf8("ЗАГРУЗКА ПИТАТЕЛЕЙ");
    }
    if (stateKey == QStringLiteral("PICKING_MATERIALS")) {
        return QString::fromUtf8("ПОДБОР МАТЕРИАЛОВ");
    }
    if (stateKey == QStringLiteral("READY_FOR_FEEDER_PREP")) {
        return QString::fromUtf8("К ЗОНЕ ПОДГОТОВКИ ПИТАТЕЛЕЙ");
    }
    if (stateKey == QStringLiteral("FEEDER_PREP")) {
        return QString::fromUtf8("ПОДГОТОВКА ПИТАТЕЛЕЙ");
    }
    if (stateKey == QStringLiteral("READY_FOR_LINE")) {
        return QString::fromUtf8("К ЛИНИИ SMT");
    }
    if (stateKey == QStringLiteral("ISSUING_TO_LINE")) {
        return QString::fromUtf8("ВЫДАЧА НА ЛИНИЮ");
    }
    if (stateKey == QStringLiteral("ORDER_COMPLETED")) {
        return QString::fromUtf8("ЗАКАЗ ЗАВЕРШЁН");
    }
    if (stateKey == QStringLiteral("LEFTOVERS_DETECTED")) {
        return QString::fromUtf8("ОБНАРУЖЕНЫ ОСТАТКИ");
    }
    if (stateKey == QStringLiteral("RETURNING_LEFTOVERS")) {
        return QString::fromUtf8("ВОЗВРАТ ОСТАТКОВ");
    }
    return {};
}

void WorkflowDialogManager::showModuleStatePopupIfNeeded(const QString& stateKey) {
    if (stateKey == QStringLiteral("FREE")) {
        lastModuleStatePopupKey_.clear();
        return;
    }
    if (!moduleOnline_) {
        return;
    }
    if (stateKey == lastModuleStatePopupKey_) {
        return;
    }

    const QString title = moduleStatePopupTitle(stateKey);
    if (title.isEmpty()) {
        return;
    }

    const QString body = viewModel_.moduleStateTextForStage(stateKey).trimmed();
    if (body.isEmpty()) {
        return;
    }

    lastModuleStatePopupKey_ = stateKey;
    if (!modalPopupsReady_) {
        return;
    }

    QTimer::singleShot(650, this, [this, stateKey, title, body]() {
        if (!modalPopupsReady_ ||
            !moduleOnline_ ||
            stateKey != lastModuleStatePopupKey_)
        {
            return;
        }
        auto* topLevel = parent_->window();
        if (!topLevel || !topLevel->isVisible() || !parent_->isVisible()) {
            lastModuleStatePopupKey_.clear();
            return;
        }
        showModuleStateDialog(title, body);
    });
}

QString WorkflowDialogManager::moduleStatePopupTitle(const QString& stateKey) {
    if (stateKey == QStringLiteral("LOADING_FEEDERS") ||
        stateKey == QStringLiteral("READY_FOR_FEEDER_PREP") ||
        stateKey == QStringLiteral("FEEDER_PREP") ||
        stateKey == QStringLiteral("READY_FOR_LINE"))
    {
        return QString::fromUtf8("СОСТОЯНИЕ МОДУЛЯ ПИТАТЕЛЕЙ");
    }

    if (stateKey == QStringLiteral("PICKING_MATERIALS") ||
        stateKey == QStringLiteral("ISSUING_TO_LINE") ||
        stateKey == QStringLiteral("LEFTOVERS_DETECTED") ||
        stateKey == QStringLiteral("RETURNING_LEFTOVERS"))
    {
        return QString::fromUtf8("СОСТОЯНИЕ МОДУЛЯ КАТУШЕК");
    }

    return {};
}

bool WorkflowDialogManager::askCartWasAlreadyInWork() {
    QDialog dialog(parent_);
    dialog.setWindowTitle(QString::fromUtf8("Состояние тележки"));
    dialog.setModal(true);
    dialog.setStyleSheet(
        "QDialog { background: #F6FAF7; }"
        "QLabel { color: #102217; background: transparent; }"
        "QPushButton { background: #E6F0E9; color: #173025; "
        "border: 1px solid #B8D0BF; border-radius: 8px; "
        "font-size: 15px; font-weight: bold; padding: 12px 16px; }"
        "QPushButton:hover { background: #D7E8DC; }"
        "QPushButton#primary { background: #2F8F57; color: #FFFFFF; "
        "border-color: #2F8F57; }"
    );

    auto* root = new QVBoxLayout(&dialog);
    root->setContentsMargins(26, 24, 26, 22);
    root->setSpacing(16);

    auto* title = new QLabel(QString::fromUtf8("СОСТОЯНИЕ ТЕЛЕЖКИ"), &dialog);
    title->setFont(QFont("Segoe UI", 22, QFont::Bold));
    title->setAlignment(Qt::AlignCenter);

    auto* text = new QLabel(
        QString::fromUtf8(
            "Если тележка пустая, оператор сначала загрузит питатели.\n"
            "Если тележка была в работе, считаем, что питатели уже находятся "
            "в тележке, и сразу переходим к подбору материалов."),
        &dialog);
    text->setFont(QFont("Segoe UI", 14));
    text->setAlignment(Qt::AlignCenter);
    text->setWordWrap(true);
    text->setMinimumWidth(620);

    auto* buttons = new QHBoxLayout();
    buttons->setSpacing(12);

    auto* inWorkButton = new QPushButton(QString::fromUtf8("Была в работе"), &dialog);
    auto* cancelButton = new QPushButton(QString::fromUtf8("Отмена"), &dialog);
    auto* emptyButton = new QPushButton(QString::fromUtf8("Тележка пустая"), &dialog);
    emptyButton->setObjectName(QStringLiteral("primary"));

    for (auto* button : {inWorkButton, cancelButton, emptyButton}) {
        button->setMinimumHeight(54);
        button->setMinimumWidth(180);
    }

    buttons->addWidget(inWorkButton);
    buttons->addWidget(cancelButton);
    buttons->addWidget(emptyButton);

    root->addWidget(title);
    root->addWidget(text);
    root->addLayout(buttons);

    bool alreadyInWork = false;
    connect(inWorkButton, &QPushButton::clicked, &dialog, [&]() {
        alreadyInWork = true;
        dialog.accept();
    });
    connect(emptyButton, &QPushButton::clicked, &dialog, [&]() {
        alreadyInWork = false;
        dialog.accept();
    });
    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) {
        return false;
    }
    return alreadyInWork;
}

void WorkflowDialogManager::showLargeStageDialog(const QString& titleText) {
    QDialog dialog(parent_);
    dialog.setWindowTitle(QString::fromUtf8("Этап маршрута"));
    dialog.setModal(true);
    dialog.setStyleSheet(
        "QDialog { background: #F6FAF7; }"
        "QLabel { color: #102217; background: transparent; }"
        "QPushButton { background: #2F8F57; color: #FFFFFF; "
        "border: 1px solid #2F8F57; border-radius: 8px; "
        "font-size: 16px; font-weight: bold; padding: 10px 18px; }"
        "QPushButton:hover { background: #3CA86A; }"
    );

    auto* root = new QVBoxLayout(&dialog);
    root->setContentsMargins(34, 30, 34, 26);
    root->setSpacing(22);

    auto* title = new QLabel(titleText, &dialog);
    title->setFont(QFont("Segoe UI", 34, QFont::Bold));
    title->setAlignment(Qt::AlignCenter);
    title->setWordWrap(true);
    title->setMinimumWidth(620);

    auto* okButton = new QPushButton(QString::fromUtf8("Понятно"), &dialog);
    okButton->setMinimumSize(180, 52);

    root->addWidget(title);
    root->addWidget(okButton, 0, Qt::AlignCenter);

    connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    dialog.exec();
}

void WorkflowDialogManager::showModuleStateDialog(const QString& titleText,
                                                   const QString& bodyText) {
    QDialog dialog(parent_);
    dialog.setWindowTitle(titleText);
    dialog.setModal(true);
    dialog.setStyleSheet(
        "QDialog { background: #F6FAF7; }"
        "QLabel { color: #102217; background: transparent; }"
        "QTextEdit { background: #FFFFFF; color: #102217; "
        "border: 1px solid #C7D8CC; border-radius: 8px; padding: 10px; }"
        "QPushButton { background: #2F8F57; color: #FFFFFF; "
        "border: 1px solid #2F8F57; border-radius: 8px; "
        "font-size: 16px; font-weight: bold; padding: 10px 18px; }"
        "QPushButton:hover { background: #3CA86A; }"
    );

    auto* root = new QVBoxLayout(&dialog);
    root->setContentsMargins(28, 24, 28, 24);
    root->setSpacing(16);

    auto* title = new QLabel(titleText, &dialog);
    title->setFont(QFont("Segoe UI", 24, QFont::Bold));
    title->setAlignment(Qt::AlignCenter);
    title->setWordWrap(true);
    title->setMinimumWidth(640);

    auto* body = new QTextEdit(&dialog);
    body->setReadOnly(true);
    body->setPlainText(bodyText);
    body->setFont(QFont("Segoe UI", 13));
    body->setMinimumSize(640, 260);

    auto* okButton = new QPushButton(QString::fromUtf8("Понятно"), &dialog);
    okButton->setMinimumSize(180, 52);

    root->addWidget(title);
    root->addWidget(body);
    root->addWidget(okButton, 0, Qt::AlignCenter);

    connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    dialog.exec();
}
