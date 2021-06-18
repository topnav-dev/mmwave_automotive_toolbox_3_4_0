function [occCount, tracker] = occupancyDetection(numZones, tracker, point3D, zoneMap, stateMachine)

    occCount = 0;
    
    for idx = 1:numZones
        % Find the number of detected points in this zone
        tracker(idx).numPoints = sum(zoneMap(:,idx));

        % Calculate the average SNR for detected points in this zone
        if (tracker(idx).numPoints > 0)
            tracker(idx).avgSnr = mean(point3D(4, 1 == zoneMap(:,idx)));
        else
            tracker(idx).avgSnr = 0.0;
        end
    end

    % Check for overload conditions (large movements in the row)
    for idx = 1:numZones
        if (tracker(idx).avgSnr >= stateMachine.overloadThreshold)

            % If an overload occurs, freeze all zones as well.
            for idx2 = 1: numZones
                tracker(idx2).freeze = 5;   % one second at 5fps
            end
            break;
        elseif (tracker(idx).freeze > 0)
            tracker(idx).freeze = tracker(idx).freeze - 1;
        end
    end

    % Update the occupancy state of each zone
    for idx = 1:numZones
        tracker(idx) = updateTrackerStateMachine(idx, tracker(idx), point3D, zoneMap, stateMachine);

        if (tracker(idx).state == 1)
            occCount = occCount + 1;
        end
    end
end


function tracker = updateTrackerStateMachine(zoneIdx, tracker, point3D, zoneMap, sm)

    if (tracker.freeze == 0)
        switch tracker.state
            case 0 % NOT_OCCUPIED
                if (((tracker.numPoints > sm.numPointForEnterThreshold1) && (tracker.avgSnr > sm.avgSnrForEnterThreshold1)) || ...
                    ((tracker.numPoints > sm.numPointForEnterThreshold2) && (tracker.avgSnr > sm.avgSnrForEnterThreshold2)))
                    tracker.state = 1;
                    tracker.detect2freeCount = 0;
                end
    
            case 1 % OCCUPIED
                if ((tracker.numPoints > sm.numPointForStayThreshold) && (tracker.avgSnr > sm.avgSnrForStayThreshold))
                    % still detected
                    tracker.detect2freeCount = 0;
                elseif (tracker.numPoints < sm.numPointToForget)
                    % Miss
                    if (tracker.detect2freeCount > sm.forgetThreshold)
                        tracker.state = 0;
                    else
                        tracker.detect2freeCount = tracker.detect2freeCount + 1;
                    end
                else
                    tracker.detect2freeCount = tracker.detect2freeCount - 1;
                end
        end
    end
end


function tracker = updateTrackerStateMachineForRow2Middle(zoneIdx, tracker, point3D, zoneMap, sm)

    % Find the number of detected points in this zone
    numPoints = sum(zoneMap(:,zoneIdx));
    % Calculate the average SNR for detected points in this zone
    if (numPoints > 0)
        avgSnr = mean(point3D(4, 1 == zoneMap(:,zoneIdx)));
        avgSnrR2Driver = mean(point3D(4, 1 == zoneMap(:,zoneIdx-1))); 
        avgSnrR2Passenger = mean(point3D(4, 1 == zoneMap(:,zoneIdx+1))); 
    else
        avgSnr = -10.0;
    end

    switch tracker.state
        case 0 % NOT_OCCUPIED
            if (((numPoints > sm.numPointForEnterThreshold1) && (avgSnr > sm.avgSnrForEnterThreshold1)) || ...
                ((numPoints > sm.numPointForEnterThreshold2) && (avgSnr > sm.avgSnrForEnterThreshold2) && ...
                 (avgSnr > avgSnrR2Driver) && (avgSnr > avgSnrR2Passenger)))
                tracker.state = 1;
                tracker.detect2freeCount = 0;
            end

        case 1 % OCCUPIED
            if ((numPoints > sm.numPointForStayThreshold) || (avgSnr > sm.avgSnrForStayThreshold))
                % detected
                tracker.detect2freeCount = 0;
            elseif (numPoints == 0)
                % Miss
                if (tracker.detect2freeCount > sm.forgetThreshold)
                    tracker.state = 0;
                else
                    tracker.detect2freeCount = tracker.detect2freeCount + 1;
                end
            else
                tracker.detect2freeCount = tracker.detect2freeCount - 1;
            end
    end
end
