#include "scriptsinterfaces.h"
#include <QFile>
#include <QApplication>

CScriptsInterfaces::CScriptsInterfaces(QObject* parent) : QObject(parent)
{
    QFile file(QApplication::applicationDirPath() + "/../lua_scripts/test/Config.json");
    if (!file.open(QIODevice::ReadWrite)) {
        qDebug() << "fail opening" << file.fileName();
        return;
    }
    qDebug() << "opening" << file.fileName();
    QJsonParseError jsonParserError;
    jsonDocument = QJsonDocument::fromJson(file.readAll(), &jsonParserError);
    if (!jsonDocument.isNull() && jsonParserError.error == QJsonParseError::NoError) {
        qDebug() << "successfully parsed" << file.fileName();
        if (jsonDocument.isObject()) {
            jsonObject = jsonDocument.object();
        }
    }
    updatePlayList();
}

void CScriptsInterfaces::updatePlayList() {
    playList.clear();
    QStringList playType = { "gTestPlay" , "gRefPlay" };
    for (int i = 0; i < playType.size(); i++) {
        if (jsonObject.contains(playType[i]) && jsonObject.value(playType[i]).isArray()) {
            QJsonArray playArray = jsonObject.value(playType[i]).toArray();
            for (int i = 0; i < playArray.size(); i++)
                playList.append(playArray[i].toString());
        }
    }
}

void CScriptsInterfaces::changeReadyPlay(int currentIndex) {
    if (currentIndex >= playList.size()) return;
    jsonObject["gReadyPlay"] = playList[currentIndex];
}

void CScriptsInterfaces::writeJsonFile() {
    // ��object����Ϊ���ĵ�����Ҫ����
    jsonDocument.setObject(jsonObject);

    // ��д���ļ�������ԭ���ļ����ﵽɾ���ļ�ȫ�����ݵ�Ч��
    QFile writeFile(QApplication::applicationDirPath() + "/../lua_scripts/test/Config.json");
    if (!writeFile.open(QFile::WriteOnly | QFile::Truncate)) {
        qDebug() << "can't open error!";
        return;
    }

    // ���޸ĺ������д���ļ�
    QTextStream wirteStream(&writeFile);
    wirteStream.setCodec("UTF-8");		// ���ö�ȡ������UTF8
    wirteStream << jsonDocument.toJson();		// д���ļ�
    writeFile.close();					// �ر��ļ�
}
