/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "settings.h"

Settings::Settings(QString settingsPath, QObject *parent) :
    QObject(parent)
{
    settings     = new QSettings(settingsPath + "/cc_plugin.ini", QSettings::IniFormat);
    // default settings
    fcSerialPort   = "COM4";
    channels.reserve(60);
    for (quint8 i = 0; i < 10; ++i) {
        inputMap << 255;
    }
    for (quint8 i = 0; i < 8; ++i) {
        outputMap << 255;
    }
    sendToRX   = true;
    takeFromTX = true;
    videoModes << 1 << 50 << 50 << 800 << 600;
}

void Settings::read()
{
    // network
    fcSerialPort = settings->value("serialport", fcSerialPort).toString();

    QString allChannels = settings->value("all_channels").toString();
    QString chan;
    int i = 0;
    foreach(chan, allChannels.split(" "))
        channels.insert(chan, i++);
    qDebug() << channels;

    // inputs
    QString num = "";
    QString map = "";
    for (quint8 i = 0; i < 10; ++i) {
        num = QString::number(i + 1);
        map = settings->value("Input/cc_channel_" + num).toString();
        inputMap[i] = channels.value(map, inputMap.at(i));
    }

    QString sendTo = settings->value("Input/send_to", "RX").toString();
    sendToRX = (sendTo == "RX") ? true : false;
    qDebug() << "sendToRX: " << sendToRX;

    // outputs
    for (quint8 i = 0; i < 8; ++i) {
        num = QString::number(i + 1);
        map = settings->value("Output/sim_channel_" + num).toString();
        outputMap[i] = channels.value(map, outputMap.at(i));
    }

    QString takeFrom = settings->value("Output/take_from", "TX").toString();
    takeFromTX = (takeFrom == "TX") ? true : false;
    qDebug() << "takeFromTX: " << takeFromTX;

    // video
    quint8 resolutionNum = settings->value("Video/number_of_resolutions", 0).toInt();
    if (resolutionNum > 0) {
        videoModes.clear();
        videoModes << resolutionNum;
        for (quint8 i = 0; i < resolutionNum; ++i) {
            num = QString::number(i + 1);
            QString modes = settings->value("Video/resolution_" + num, "0, 0, 640, 480").toString();
            QString mode;
            foreach(mode, modes.split(" "))
            videoModes << mode.toInt();
        }
    }
}
