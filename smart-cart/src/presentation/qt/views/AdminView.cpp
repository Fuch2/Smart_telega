#include "AdminView.hpp"
#include "../viewmodels/AdminViewModel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QHeaderView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QAbstractItemView>
#include <QBrush>
#include <QColor>

AdminView::AdminView(QWidget* parent) : QWidget(parent) {
    buildUi();
    bindVm();
    vm_->loadDemo();
}

void AdminView::buildUi() {
    auto* root = new QVBoxLayout(this);

    auto* title = new QLabel("AdminView: управление модулями", this);
    msgLabel_ = new QLabel("-", this);

    table_ = new QTableWidget(this);
    table_->setColumnCount(5);
    table_->setHorizontalHeaderLabels({"ID", "Serial", "Slots", "Firmware", "Status"});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

    auto* form = new QGridLayout();
    serialEdit_ = new QLineEdit(this);
    slotsSpin_ = new QSpinBox(this);
    slotsSpin_->setRange(1, 96);
    slotsSpin_->setValue(24);
    fwEdit_ = new QLineEdit(this);
    statusCombo_ = new QComboBox(this);
    statusCombo_->addItems({"ONLINE", "OFFLINE", "MAINT"});

    form->addWidget(new QLabel("Serial:", this), 0, 0);
    form->addWidget(serialEdit_, 0, 1);
    form->addWidget(new QLabel("Slots:", this), 0, 2);
    form->addWidget(slotsSpin_, 0, 3);
    form->addWidget(new QLabel("Firmware:", this), 1, 0);
    form->addWidget(fwEdit_, 1, 1);
    form->addWidget(new QLabel("Status:", this), 1, 2);
    form->addWidget(statusCombo_, 1, 3);

    auto* btns = new QHBoxLayout();
    addBtn_ = new QPushButton("Добавить", this);
    updateBtn_ = new QPushButton("Обновить", this);
    deleteBtn_ = new QPushButton("Удалить", this);
    reloadBtn_ = new QPushButton("Перезагрузить demo", this);

    btns->addWidget(addBtn_);
    btns->addWidget(updateBtn_);
    btns->addWidget(deleteBtn_);
    btns->addStretch(1);
    btns->addWidget(reloadBtn_);

    root->addWidget(title);
    root->addWidget(msgLabel_);
    root->addWidget(table_, 1);
    root->addLayout(form);
    root->addLayout(btns);
}

void AdminView::bindVm() {
    vm_ = new AdminViewModel(this);

    connect(vm_, &AdminViewModel::modulesReset, this, &AdminView::refreshTable);

    connect(vm_, &AdminViewModel::errorOccurred, this, [this](const QString& m) {
        msgLabel_->setText("Ошибка: " + m);
        msgLabel_->setStyleSheet("color:#ff6b6b;");
    });

    connect(vm_, &AdminViewModel::infoOccurred, this, [this](const QString& m) {
        msgLabel_->setText("OK: " + m);
        msgLabel_->setStyleSheet("color:#7bd88f;");
    });

    connect(table_, &QTableWidget::cellClicked, this, [this](int row, int) {
        loadRowToForm(row);
    });

    connect(addBtn_, &QPushButton::clicked, this, [this]() {
        const QString serial = serialEdit_->text().trimmed();
        const int slotCount = slotsSpin_->value();
        const QString fw = fwEdit_->text().trimmed();
        const QString st = statusCombo_->currentText();

        vm_->addModule(serial, slotCount, fw, st);

        // Автовыбор добавленной строки по serial (достаточно для demo)
        const auto& items = vm_->items();
        for (int i = items.size() - 1; i >= 0; --i) {
            if (items[i].serial.compare(serial, Qt::CaseInsensitive) == 0) {
                selectRowById(items[i].id);
                break;
            }
        }
    });

    connect(updateBtn_, &QPushButton::clicked, this, [this]() {
        if (selectedId_ == 0) {
            msgLabel_->setText("Ошибка: выбери строку для обновления");
            msgLabel_->setStyleSheet("color:#ff6b6b;");
            return;
        }

        vm_->updateModule(
            selectedId_,
            serialEdit_->text().trimmed(),
            slotsSpin_->value(),
            fwEdit_->text().trimmed(),
            statusCombo_->currentText()
        );

        // Сохраняем выделение текущей записи
        selectRowById(selectedId_);
    });

    connect(deleteBtn_, &QPushButton::clicked, this, [this]() {
        if (selectedId_ == 0) {
            msgLabel_->setText("Ошибка: выбери строку для удаления");
            msgLabel_->setStyleSheet("color:#ff6b6b;");
            return;
        }

        vm_->removeModule(selectedId_);
        selectedId_ = 0;
        resetForm(); // <-- полировка после удаления
    });

    connect(reloadBtn_, &QPushButton::clicked, this, [this]() {
        vm_->loadDemo();
        selectedId_ = 0;
        resetForm();
    });
}

void AdminView::refreshTable() {
    const auto& items = vm_->items();
    table_->setRowCount(items.size());

    for (int r = 0; r < items.size(); ++r) {
        const auto& m = items[r];

        auto* idIt = new QTableWidgetItem(QString::number(m.id));
        auto* sIt  = new QTableWidgetItem(m.serial);
        auto* slIt = new QTableWidgetItem(QString::number(m.slotCount));
        auto* fwIt = new QTableWidgetItem(m.firmware);
        auto* stIt = new QTableWidgetItem(m.status);

        // Цвет статуса
        if (m.status == "ONLINE") {
            stIt->setForeground(QBrush(QColor("#7bd88f")));
        } else if (m.status == "OFFLINE") {
            stIt->setForeground(QBrush(QColor("#ff6b6b")));
        } else if (m.status == "MAINT") {
            stIt->setForeground(QBrush(QColor("#ffd866")));
        }

        table_->setItem(r, 0, idIt);
        table_->setItem(r, 1, sIt);
        table_->setItem(r, 2, slIt);
        table_->setItem(r, 3, fwIt);
        table_->setItem(r, 4, stIt);
    }
}

void AdminView::loadRowToForm(int row) {
    if (row < 0) return;

    auto* idItem = table_->item(row, 0);
    auto* sItem  = table_->item(row, 1);
    auto* slItem = table_->item(row, 2);
    auto* fwItem = table_->item(row, 3);
    auto* stItem = table_->item(row, 4);
    if (!idItem || !sItem || !slItem || !fwItem || !stItem) return;

    selectedId_ = idItem->text().toInt();
    serialEdit_->setText(sItem->text());
    slotsSpin_->setValue(slItem->text().toInt());
    fwEdit_->setText(fwItem->text());

    const int idx = statusCombo_->findText(stItem->text());
    if (idx >= 0) statusCombo_->setCurrentIndex(idx);
}

void AdminView::resetForm() {
    serialEdit_->clear();
    fwEdit_->clear();
    slotsSpin_->setValue(24);
    statusCombo_->setCurrentText("ONLINE");
}

void AdminView::selectRowById(int id) {
    for (int r = 0; r < table_->rowCount(); ++r) {
        auto* idItem = table_->item(r, 0);
        if (!idItem) continue;
        if (idItem->text().toInt() == id) {
            table_->selectRow(r);
            loadRowToForm(r);
            return;
        }
    }
}
