//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "TabTargets.h"

//-----------------------------------------------------------------------------
// UI Setup Information
//-----------------------------------------------------------------------------
#ifndef dimensionof
#define dimensionof(a) (sizeof(a) / sizeof((a)[0]))
#endif

struct tTargetFieldInfo {
  const char* name;
  unsigned width;
  Qt::Alignment alignment;
};

const tTargetFieldInfo targetFieldInfo[tTabTargets::cMaxTargetFields] = {
    {
        "ServerID",
        70,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "ClientID",
        70,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "State",
        180,
        Qt::AlignCenter,
    },
    {
        "Range (m)",
        100,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "Bearing (deg)",
        100,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "Course (deg)",
        100,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "Speed (m/s)",
        100,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "CPA (m)",
        110,
        Qt::AlignRight | Qt::AlignVCenter,
    },
    {
        "TCPA",
        110,
        Qt::AlignRight | Qt::AlignVCenter,
    },
};

const char* targetStateNames[] = {
    "Acquiring",                 // 0x00
    "Safe Target",               // 0x01
    "Dangerous Target",          // 0x02
    "Lost Target",               // 0x03
    "Acquire Failure",           // 0x04
    "Out of Range",              // 0x05
    "Lost - Out of Range",       // 0x06
    "Invalid State (7)",         // 0x07
    "Invalid State (8)",         // 0x08
    "Invalid State (9)",         // 0x09
    "Invalid State (10)",        // 0x0a
    "Invalid State (11)",        // 0x0b
    "Invalid State (12)",        // 0x0c
    "Invalid State (13)",        // 0x0d
    "Invalid State (14)",        // 0x0e
    "Invalid State (15)",        // 0x0f
    "Failed - Max Targets",      // 0x10
    "Failed - Invalid Position"  // 0x11
};

//-----------------------------------------------------------------------------
QString FormatTime(uint32_t secs) {
  int hours = secs / (60 * 60);
  secs -= (60 * 60) * hours;
  int mins = secs / 60;
  secs -= 60 * mins;
  return QString("%0:%1:%2")
      .arg(hours)
      .arg(mins, 2, 10, QChar('0'))
      .arg(secs, 2, 10, QChar('0'));
}

//-----------------------------------------------------------------------------
// tTabTargets Implementation
//-----------------------------------------------------------------------------
tTabTargets::tTabTargets(Ui::GUIDemoClass& ui, QObject* pParent, QWidget& tab)
    : tTabBase(ui, pParent, tab), m_pTargetTrackingClient(nullptr) {
  ConnectControls(true, *this, tab);

  // setup target table headers
  m_Ui.tableTargets->setCornerButtonEnabled(false);
  m_Ui.tableTargets->setSelectionBehavior(QTableWidget::SelectRows);
  m_Ui.tableTargets->setSelectionMode(QTableWidget::ExtendedSelection);
  m_Ui.tableTargets->setColumnCount(cMaxTargetFields);
  m_Ui.tableTargets->setRowCount(cMaxTargets + 1);
  for (int i = 0; i < cMaxTargetFields; ++i) {
    QTableWidgetItem* pItem = new QTableWidgetItem(targetFieldInfo[i].name);
    pItem->setTextAlignment(Qt::AlignCenter);
    m_Ui.tableTargets->setHorizontalHeaderItem(i, pItem);
    m_Ui.tableTargets->setColumnWidth(i, targetFieldInfo[i].width);
  }
  for (unsigned i = 0; i < cMaxTargets; ++i) {
    m_Ui.tableTargets->setRowHeight(i, 25);
  }
  m_Ui.tableTargets->verticalHeader()->setVisible(false);

  Connect(true, m_Ui.tableTargets, SIGNAL(itemSelectionChanged()), this,
          SLOT(Table_itemSelectionChanged()));
}

//-----------------------------------------------------------------------------
tTabTargets::~tTabTargets() {
  ConnectControls(false, *this, m_Tab);
  Connect(false, m_Ui.tableTargets, SIGNAL(itemSelectionChanged()), this,
          SLOT(Table_itemSelectionChanged()));
}

//-----------------------------------------------------------------------------
void tTabTargets::OnConnect(
    Navico::Protocol::NRP::tTargetTrackingClient* pTargetTrackingClient) {
  m_pTargetTrackingClient = pTargetTrackingClient;
  m_Ui.tabTargets->setEnabled(m_pTargetTrackingClient != nullptr);
  m_UpdateAlarmSetupControls = true;
}

//-----------------------------------------------------------------------------
void tTabTargets::OnDisconnect() {
  m_Ui.tabTargets->setEnabled(false);
  m_pTargetTrackingClient = nullptr;
}

//-----------------------------------------------------------------------------
//  UI Events
//-----------------------------------------------------------------------------
void tTabTargets::DangerZoneDist_valueChanged(int value) {
  if (m_pTargetTrackingClient)
    m_pTargetTrackingClient->SetDangerDistance(static_cast<uint32_t>(value));
}

//-----------------------------------------------------------------------------
void tTabTargets::DangerZoneTime_valueChanged(int value) {
  if (m_pTargetTrackingClient) m_pTargetTrackingClient->SetDangerTime(static_cast<uint32_t>(value));
}

//-----------------------------------------------------------------------------
void tTabTargets::TargetsQueryAll_clicked(bool /*checked*/) {
  m_pTargetTrackingClient->QueryAll();
}

//-----------------------------------------------------------------------------
void tTabTargets::TargetsQuerySetup_clicked(bool /*checked*/) {
  m_pTargetTrackingClient->QueryAlarmSetup();
}

//-----------------------------------------------------------------------------
void tTabTargets::TargetsQueryTargets_clicked(bool /*checked*/) {
  m_pTargetTrackingClient->QueryAllTargets();
}

//-----------------------------------------------------------------------------
void tTabTargets::CancelAll_clicked(bool /*checked*/) {
  m_pTargetTrackingClient->CancelAll();
}

//-----------------------------------------------------------------------------
tTabTargets::tServerIDs tTabTargets::GetSelectedTargets() {
  tServerIDs serverIDs;

  const QList<QTableWidgetItem*> list(m_Ui.tableTargets->selectedItems());
  int count = list.size();
  if (count > 0) {
    for (int i = 0; i < count; ++i) {
      int serverID = list[i]->row();
      if (serverID > 0 && unsigned(serverID) <= 1 + cMaxTargets) {
        // if (m_pTargetLocations[ serverID-1 ].isValid)
        serverIDs.append(serverID);
      }
    }
  }
  return serverIDs;
}

//-----------------------------------------------------------------------------
void tTabTargets::CancelSelection_clicked(bool /*checked*/) {
  tServerIDs serverIDs(GetSelectedTargets());
  m_Ui.tableTargets->clearSelection();
  for (tServerIDs::const_iterator it = serverIDs.begin(); it != serverIDs.end();
       ++it) {
    m_pTargetTrackingClient->Cancel(*it);
  }
}

//-----------------------------------------------------------------------------
void tTabTargets::CancelDisplay_clicked(bool /*checked*/) {
  for (unsigned targetIndex = 0; targetIndex <= cMaxTargets; ++targetIndex) {
    for (int i = 0; i < cMaxTargetFields; ++i) {
      TabTargetSetFieldText(targetIndex, eTargetFields(i), QString());
    }
  }
}

//-----------------------------------------------------------------------------
void tTabTargets::Table_itemSelectionChanged() {
  const QList<QTableWidgetItem*> list(m_Ui.tableTargets->selectedItems());
  m_Ui.pushCancelSelection->setEnabled(!GetSelectedTargets().isEmpty());
}

//-----------------------------------------------------------------------------
//  Target Tracking Service Events
//-----------------------------------------------------------------------------
void tTabTargets::OnTTAlarmSetupChanged(
    const Navico::Protocol::NRP::tTargetTrackingAlarmSetup* pAlarmSetup) {
  m_Ui.editDangerZoneDist->setText(
      QString::number(pAlarmSetup->safeZoneDistance_m));
  m_Ui.editDangerZoneTime->setText(
      QString::number(pAlarmSetup->safeZoneTime_sec));

  if (m_UpdateAlarmSetupControls) {
    m_UpdateAlarmSetupControls = false;

    m_Ui.spinDangerZoneDist->setValue(pAlarmSetup->safeZoneDistance_m);
    m_Ui.spinDangerZoneTime->setValue(pAlarmSetup->safeZoneTime_sec);
  }
}

//-----------------------------------------------------------------------------
void tTabTargets::OnTTPropertiesChanged(
    const Navico::Protocol::NRP::tTargetTrackingProperties* pProperties) {

}

//-----------------------------------------------------------------------------
void tTabTargets::OnTrackedTargetChanged(
    unsigned targetIndex,
    const Navico::Protocol::NRP::tTrackedTarget* pTarget) {
  if (pTarget->targetValid) {
    unsigned state = pTarget->targetState;
    QString stateStr;
    if (state < dimensionof(targetStateNames))
      stateStr = targetStateNames[state];
    else if (state == Navico::Protocol::NRP::eBadState)
      stateStr = "Bad-State";
    else
      stateStr = QString("Invalid State (%0)").arg(state);

    TabTargetSetFieldText(targetIndex, TField_State, stateStr);
    TabTargetSetFieldText(targetIndex, TField_ServerID,
                          QString::number(pTarget->serverTargetID));
    TabTargetSetFieldText(targetIndex, TField_ClientID,
                          QString::number(pTarget->targetID));
    TabTargetSetFieldText(targetIndex, TField_RelRange,
                          QString::number(pTarget->infoRelative.distance_m));
    TabTargetSetFieldText(
        targetIndex, TField_RelBearing,
        QString::number(pTarget->infoRelative.bearing_ddeg / 10.0, 'f', 1));
    TabTargetSetFieldText(
        targetIndex, TField_RelCourse,
        QString::number(pTarget->infoRelative.course_ddeg / 10.0, 'f', 1));
    TabTargetSetFieldText(
        targetIndex, TField_RelSpeed,
        QString::number(pTarget->infoRelative.speed_dmps / 10.0, 'f', 1));
    TabTargetSetFieldText(targetIndex, TField_CPA,
                          QString::number(pTarget->CPA_m));
    TabTargetSetFieldText(
        targetIndex, TField_TCPA,
        pTarget->towardsCPA ? FormatTime(pTarget->TCPA_sec) : "");
  } else {
    for (int i = 0; i < cMaxTargetFields; ++i) {
      TabTargetSetFieldText(targetIndex, eTargetFields(i), QString());
    }
  }
}

//-----------------------------------------------------------------------------
void tTabTargets::TabTargetSetFieldText(unsigned targetID, eTargetFields field,
                                        const QString& text) {
  QTableWidgetItem* pItem = m_Ui.tableTargets->item(targetID, field);
  if (pItem == nullptr) {
    pItem = new QTableWidgetItem();
    pItem->setTextAlignment(targetFieldInfo[field].alignment);
    m_Ui.tableTargets->setItem(targetID, field, pItem);
  }
  pItem->setText(text);
}
