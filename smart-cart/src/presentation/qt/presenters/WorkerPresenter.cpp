// ===== src/presentation/qt/presenters/WorkerPresenter.cpp =====
#include "WorkerPresenter.hpp"
#include "../viewmodels/WorkerViewModel.hpp"

WorkerPresenter::WorkerPresenter(WorkerViewModel* vm, QObject* parent)
    : QObject(parent)
    , vm_(vm)
{}

void WorkerPresenter::injectBarcode(const QString& barcode) {
    if (vm_) vm_->submitBarcode(barcode);
}
