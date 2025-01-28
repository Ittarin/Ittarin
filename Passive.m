clc; clear; close all;

% MATLABサーバーの設定（Python側と一致させる）
MATLAB_PORT = 54321;  % Pythonと同じポート番号
server = tcpserver("0.0.0.0", MATLAB_PORT, "Timeout", 30);  % TCPサーバーを作成

disp("MATLABサーバーが接続待機中...");

% データ記録用
angles = [];
errors = [];
speeds = [];
torques = [];
temperatures = [];
times = [];
start_time = tic;

% 図の設定
figure;
hold on;
grid on;
xlabel("Time (s)");
ylabel("Angle & Error (rad)");
title("Angle & Error vs Time");

% 初期プロット
h1 = plot(NaN, NaN, 'b', 'DisplayName', 'Angle (rad)'); % 角度プロット（青）
h2 = plot(NaN, NaN, 'r', 'DisplayName', 'Error (rad)');  % 誤差プロット（赤）
legend; % プロットが存在するので凡例を追加

while true
    try
        % データの受信
        if server.NumBytesAvailable > 0
            data = readline(server);  % 1行データを取得
            values = str2double(strsplit(data, ','));  % カンマで分割し数値化
            
            if length(values) == 5
                angle = values(1);       % 角度 (rad)
                error = values(2);       % 誤差 (rad)
                speed = values(3);       % 速度 (RPM)
                torque = values(4);      % トルク電流
                temp = values(5);        % 温度 (°C)
                
                % 時間の記録
                elapsed_time = toc(start_time);
                times = [times, elapsed_time];
                
                % データ保存
                angles = [angles, angle];
                errors = [errors, error];
                speeds = [speeds, speed];
                torques = [torques, torque];
                temperatures = [temperatures, temp];
                
                % 受信データの表示
                fprintf("Time: %.2f s, Angle: %.4f rad, Error: %.4f rad, Speed: %.2f RPM, Torque: %d, Temp: %d °C\n", ...
                        elapsed_time, angle, error, speed, torque, temp);
                
                % グラフ更新
                set(h1, 'XData', times, 'YData', angles);
                set(h2, 'XData', times, 'YData', errors);
                drawnow;
            end
        end
    catch ME
        warning("通信エラー: %s", ME.message);
        break;
    end
end

% クリーンアップ
clear server;
disp("MATLABサーバーを終了しました");
