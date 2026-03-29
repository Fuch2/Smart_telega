/****************************************************************************
** Meta object code from reading C++ file 'WorkerViewModel.hpp'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.9.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/presentation/qt/viewmodels/WorkerViewModel.hpp"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'WorkerViewModel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.9.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN15WorkerViewModelE_t {};
} // unnamed namespace

template <> constexpr inline auto WorkerViewModel::qt_create_metaobjectdata<qt_meta_tag_ZN15WorkerViewModelE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "WorkerViewModel",
        "rfidStatusChanged",
        "",
        "v",
        "uartStatusChanged",
        "scannerStatusChanged",
        "moduleSerialChanged",
        "slotOccupiedChanged",
        "slotIndex",
        "occupied",
        "targetSlotChanged",
        "onTick"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'rfidStatusChanged'
        QtMocHelpers::SignalData<void(const QString &)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QString, 3 },
        }}),
        // Signal 'uartStatusChanged'
        QtMocHelpers::SignalData<void(const QString &)>(4, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QString, 3 },
        }}),
        // Signal 'scannerStatusChanged'
        QtMocHelpers::SignalData<void(const QString &)>(5, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QString, 3 },
        }}),
        // Signal 'moduleSerialChanged'
        QtMocHelpers::SignalData<void(const QString &)>(6, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QString, 3 },
        }}),
        // Signal 'slotOccupiedChanged'
        QtMocHelpers::SignalData<void(int, bool)>(7, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 8 }, { QMetaType::Bool, 9 },
        }}),
        // Signal 'targetSlotChanged'
        QtMocHelpers::SignalData<void(int)>(10, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 8 },
        }}),
        // Slot 'onTick'
        QtMocHelpers::SlotData<void()>(11, 2, QMC::AccessPrivate, QMetaType::Void),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<WorkerViewModel, qt_meta_tag_ZN15WorkerViewModelE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject WorkerViewModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN15WorkerViewModelE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN15WorkerViewModelE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN15WorkerViewModelE_t>.metaTypes,
    nullptr
} };

void WorkerViewModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<WorkerViewModel *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->rfidStatusChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->uartStatusChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->scannerStatusChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->moduleSerialChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->slotOccupiedChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2]))); break;
        case 5: _t->targetSlotChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 6: _t->onTick(); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (WorkerViewModel::*)(const QString & )>(_a, &WorkerViewModel::rfidStatusChanged, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (WorkerViewModel::*)(const QString & )>(_a, &WorkerViewModel::uartStatusChanged, 1))
            return;
        if (QtMocHelpers::indexOfMethod<void (WorkerViewModel::*)(const QString & )>(_a, &WorkerViewModel::scannerStatusChanged, 2))
            return;
        if (QtMocHelpers::indexOfMethod<void (WorkerViewModel::*)(const QString & )>(_a, &WorkerViewModel::moduleSerialChanged, 3))
            return;
        if (QtMocHelpers::indexOfMethod<void (WorkerViewModel::*)(int , bool )>(_a, &WorkerViewModel::slotOccupiedChanged, 4))
            return;
        if (QtMocHelpers::indexOfMethod<void (WorkerViewModel::*)(int )>(_a, &WorkerViewModel::targetSlotChanged, 5))
            return;
    }
}

const QMetaObject *WorkerViewModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WorkerViewModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN15WorkerViewModelE_t>.strings))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int WorkerViewModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void WorkerViewModel::rfidStatusChanged(const QString & _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 0, nullptr, _t1);
}

// SIGNAL 1
void WorkerViewModel::uartStatusChanged(const QString & _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 1, nullptr, _t1);
}

// SIGNAL 2
void WorkerViewModel::scannerStatusChanged(const QString & _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 2, nullptr, _t1);
}

// SIGNAL 3
void WorkerViewModel::moduleSerialChanged(const QString & _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 3, nullptr, _t1);
}

// SIGNAL 4
void WorkerViewModel::slotOccupiedChanged(int _t1, bool _t2)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 4, nullptr, _t1, _t2);
}

// SIGNAL 5
void WorkerViewModel::targetSlotChanged(int _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 5, nullptr, _t1);
}
QT_WARNING_POP
